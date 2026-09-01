/*
 * Protocol-compliant telemetry (SmartWagon Telemetry Protocol Rev.1, pv=1).
 * Builds the common envelope + type payload and publishes on the correct
 * per-wagon topic. seq is monotonic per gateway (persist in FRAM in prod).
 */

#include <zephyr/kernel.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "telemetry.h"
#include "app_config.h"
#include "gnss.h"
#include "ec200.h"
#include "ble_sensors.h"
#include "nodes.h"
#include "gwalarm.h"     /* gwalarm_band() - s7.10 condition band per node */
#include "sht40.h"
#include "storage.h"
#include "config.h"
#include "charger.h"
#include "power.h"   /* power_mux_is_lto() */
#include "thermal.h"
#include "modem_simple.h"

/* Publish; if the link is down, buffer the record to the NOR archive. */
static int telem_send(const char *topic, const char *msg)
{
	/*
	 * Thermal halt (s5.1.10) is checked BEFORE the publish attempt, not
	 * after: the point is to keep the modem off, and a failed publish still
	 * powers the radio. Falling through to the buffer path below means a
	 * halted gateway behaves exactly like an out-of-coverage one, which is
	 * already the well-tested path - nothing is lost, only deferred.
	 */
	/*
	 * Publish through modem_simple, not ec200_mqtt_publish().
	 *
	 * modem_simple.c is all_hw_test's EC200 path copied verbatim - the only
	 * code that has ever attached and published from this board. ec200.c's
	 * own path has never received a byte from the module here, despite the
	 * same driver working on the DK, and repeated attempts to find the
	 * difference by inspection did not converge.
	 *
	 * ec200.c is still used for the HTTP/OTA download and the downlink URC
	 * queue; only the publish path moved.
	 */
	if (!thermal_tx_blocked() && modem_simple_publish(topic, msg) == 0) {
		return 0;
	}
	/* Offline: buffer to the NOR ring. Alarms/events are CRITICAL - the ring
	 * protects them from being overwritten by later heartbeats when full. */
	int critical = (strstr(topic, "/alarm") != NULL) ||
		       (strstr(topic, "/event") != NULL);
	int rc = storage_append(msg, (int)strlen(msg), critical);

	if (rc != 0) {
		/*
		 * The record is now LOST - it failed to publish and failed to
		 * buffer. Never let that pass silently: -ENOSPC means the ring is
		 * full of alarms (by design, and worth knowing), anything else is a
		 * flash fault or an oversized record and needs investigating.
		 */
		printk("TELEM: RECORD LOST - buffer failed (%d, %u bytes, %s)\n",
		       rc, (unsigned)strlen(msg),
		       critical ? "critical" : "heartbeat");
	}
	return -1;
}

/* Replay buffered records oldest-first on reconnect (original payloads). */
void telem_flush_backlog(void)
{
	/* Must be >= the largest buffered record (a full heartbeat ~2.4 KB). */
	static char rec[3072];
	while (storage_count() > 0) {
		int n = storage_peek(rec, sizeof(rec));
		if (n <= 0) {
			break;
		}
		char kind[12] = "hb";
		char *p = strstr(rec, "\"mt\":\"");
		if (p) {
			p += 6; int i = 0;
			while (p[i] && p[i] != '"' && i < 11) { kind[i] = p[i]; i++; }
			kind[i] = '\0';
		}
		char t[96];
		snprintf(t, sizeof(t), "%s/%s/%s/up/%s",
			 TOPIC_ROOT, cfg_wagon(), cfg_gw_id(), kind);
		/*
		 * modem_simple_publish, NOT ec200_mqtt_publish.
		 *
		 * telem_send() moved to modem_simple because ec200.c's publish
		 * path has never received a byte from the module on this board.
		 * This function was left behind on the old path, so every live
		 * record published and every REPLAYED record failed - the ring
		 * filled, the flush broke out of the loop on the first item every
		 * time, and the backlog could never drain. Store-and-forward
		 * looked implemented and was, in practice, write-only.
		 */
		if (modem_simple_publish(t, rec) == 0) {
			storage_pop();          /* remove only after PUBACK */
		} else {
			break;                  /* link lost - keep for later */
		}
	}
}

static uint32_t g_seq;   /* last uplink seq (from config_next_seq) for aid refs */

/* -------- software clock: disciplined by GNSS UTC, settable via time_sync ----
 * Free-runs off k_uptime between fixes so `ts` is valid even without a current
 * fix. (Resets on power loss until the next fix / time_sync - a hardware RTC on
 * a backup domain would persist across resets; that is a future addition.) */
static int64_t g_epoch_base;    /* epoch seconds at g_uptime_base */
static int64_t g_uptime_base;   /* k_uptime_get() ms at the base  */
static bool    g_time_set;

void telem_set_time(uint32_t epoch)
{
	g_epoch_base  = epoch;
	g_uptime_base = k_uptime_get();
	g_time_set    = true;
}

/* -------- UTC epoch from GNSS ISO "YYYY-MM-DDThh:mm:ssZ" -------- */
static uint32_t iso_to_epoch(const char *s)
{
	int Y,Mo,D,h,mi,se;
	if (!s || sscanf(s, "%d-%d-%dT%d:%d:%dZ", &Y,&Mo,&D,&h,&mi,&se) != 6) {
		return 0;
	}
	/* days from civil (Howard Hinnant) */
	int y = Y - (Mo <= 2);
	long era = (y >= 0 ? y : y - 399) / 400;
	unsigned yoe = (unsigned)(y - era * 400);
	unsigned doy = (153u * (Mo + (Mo > 2 ? -3 : 9)) + 2) / 5 + D - 1;
	unsigned doe = yoe * 365 + yoe/4 - yoe/100 + doy;
	long days = era * 146097L + (long)doe - 719468L;
	return (uint32_t)(days * 86400L + h*3600 + mi*60 + se);
}

uint32_t telem_epoch(void)
{
	struct gnss_fix f; gnss_get(&f);
	uint32_t g = iso_to_epoch(f.utc_iso);
	if (g != 0) {
		telem_set_time(g);        /* discipline the software clock from GNSS */
		return g;
	}
	if (g_time_set) {             /* no fix now: free-run from the last set time */
		return (uint32_t)(g_epoch_base +
				  (k_uptime_get() - g_uptime_base) / 1000);
	}
	return 0;
}

/* -------- compact power summary (real BQ25798 read via charger.c) -------- */
/*
 * Active rail, per Rev.1 s2.2: "read from the power-mux status, NOT inferred".
 *
 * The TPS2116 decides in hardware from its own PR threshold; power_mux_is_lto()
 * just reads its ST pin. The SoC estimate survives ONLY as a fallback for a pin
 * that cannot be read - and it is a poor one: on the bench a 3.7 V supply made
 * the estimate say "bkp" on a board with no backup fitted, which would have
 * been dead if it were true.
 */
static const char *active_src(const struct charger_status *cs)
{
	bool known = false;
	bool lto = power_mux_is_lto(&known);

	if (known) {
		return lto ? "lto" : "bkp";
	}
	/*
	 * Fallback estimate, on VOLTAGE not SoC. The percentage depends on the
	 * full/empty map being right; the voltage does not, and it is the same
	 * quantity the mux itself decides on.
	 */
	return (cs->vbus_present || cs->vbat_mv > LTO_SWITCHOVER_MV)
		       ? "lto" : "bkp";
}

static void pwr_compact(char *o, size_t n)
{
	struct charger_status cs;
	if (charger_read(&cs) == 0) {
		/*
		 * src is DERIVED, not sensed - the same rule gwalarm.c uses, so
		 * the two cannot disagree. The board has a MUX_ST line (P1.11)
		 * that would measure it properly; nothing reads it yet.
		 *
		 * bkp is "na" because NOTHING ON THIS BOARD MEASURES THE BACKUP
		 * CELL - there is no ADC on LiSOCl2_BAT and no field for it in
		 * charger_status. It used to report "ok" unconditionally, which
		 * asserted a healthy backup on a gateway that has none fitted.
		 */
		/*
		 * sol is the SOLAR INPUT state - off | present | charging - not
		 * the charger state. It used to carry charger_chg_str(), so this
		 * field published "idle", "taper", "precharge"... none of which
		 * are legal values for it. The charger state has its own home in
		 * pwr2.lto.chg, where it was already correct.
		 *
		 * 'present' and 'charging' are deliberately distinct: in low light
		 * or cold the panel can supply VBUS while below its
		 * charging-operation threshold, and an operator needs to tell
		 * "panel connected but not delivering" from "panel dead".
		 */
		const char *sol = !cs.vbus_present ? "off"
				: (cs.ibat_ma > 0 ? "charging" : "present");

		snprintf(o, n, "\"pwr\":{\"src\":\"%s\",\"soc\":%u,"
			       "\"sol\":\"%s\",\"bkp\":\"na\"}",
			 active_src(&cs), cs.soc, sol);
	} else {
		snprintf(o, n, "\"pwr\":{\"src\":\"lto\",\"soc\":0,"
			       "\"sol\":\"unknown\",\"bkp\":\"na\"}");
	}
}

/* -------- envelope header (returns bytes written) -------- */
static int envelope(char *o, size_t n, const char *mt)
{
	struct gnss_fix f; gnss_get(&f);
	char pwr[96]; pwr_compact(pwr, sizeof(pwr));
	char sysj[72]; gnss_sys_json(sysj, sizeof(sysj));

	int len = snprintf(o, n,
		"{\"mt\":\"%s\",\"pv\":%d,\"seq\":%u,\"gw\":\"%s\",\"wgn\":\"%s\","
		"\"ts\":%u,\"loc\":{\"lat\":%.6f,\"lon\":%.6f,\"cep\":%u,"
		"\"sys\":%s},\"spd\":%.0f,%s,\"d\":",
		mt, PROTO_PV, (g_seq = config_next_seq()), cfg_gw_id(), cfg_wagon(),
		telem_epoch(), f.lat_deg, f.lon_deg, gnss_cep_m(&f), sysj, f.speed_kmh, pwr);
	return len;
}

/* -------- topic helper -------- */
static void topic(char *o, size_t n, const char *kind)
{
	snprintf(o, n, "%s/%s/%s/up/%s", TOPIC_ROOT, cfg_wagon(), cfg_gw_id(), kind);
}

/* map (type,id) -> node serial like "NODE-BRG-03" */
static void node_serial(char *o, size_t n, uint8_t type, uint8_t id)
{
	const char *p = "GEN";
	switch (type) {
	case SW_TYPE_BEARING:   p = "BRG"; break;
	case SW_TYPE_LOAD_TILT: p = "LOAD"; break;
	case SW_TYPE_HANDBRAKE: p = "HB"; break;
	case SW_TYPE_TANK_TEMP: p = "TTMP"; break;
	case SW_TYPE_DOOR:      p = "DOOR"; break;
	case SW_TYPE_BRAKE:     p = "BRK"; break;
	}
	snprintf(o, n, "NODE-%s-%02d", p, id);
}

/* physical-position label for a node id (from the roster), "" if unknown */
static const char *node_pos(uint8_t id)
{
	for (int i = 0; i < WAGON_NODE_COUNT; i++) {
		if (WAGON_NODES[i].id == id) {
			return WAGON_NODES[i].pos ? WAGON_NODES[i].pos : "";
		}
	}
	return "";
}

void telem_init(void) { g_seq = 0; g_time_set = false; }

/* ---------------- HEARTBEAT (up/hb, §5) ---------------- */
int telem_pub_heartbeat(const char *mode)
{
	/* Sized for the full node roster: ~0.6 KB envelope+head+tail plus ~100 B
	 * per sensor. 3 KB holds ~24 nodes and still fits one 4 KB flash slot. */
	static char msg[3072];
	struct gnss_fix f; gnss_get(&f);

	/* local climate (SHT40) - powers its own rail on/off */
	int16_t envt = 0; uint8_t envh = 0;
	(void)sht40_read(&envt, &envh);

	char hbsys[72]; gnss_sys_json(hbsys, sizeof(hbsys));
	int len = envelope(msg, sizeof(msg), "hb");
	len += snprintf(msg+len, sizeof(msg)-len,
		"{\"mode\":\"%s\",\"mot\":{\"spd\":%.0f,\"hdg\":%.0f},"
		"\"env\":{\"t\":%d.%d,\"h\":%d},"
		"\"gnss\":{\"sys\":%s,\"fix\":\"%s\","
		"\"nsat\":%u,\"hdop\":%.1f},\"sens\":[",
		mode, f.speed_kmh, f.course_deg,
		envt/10, (envt<0?-envt:envt)%10, envh,
		hbsys, f.valid ? (f.fix_q >= 1 ? "3d" : "2d") : "none",
		f.nsat, (double)f.hdop);

	/*
	 * Iterate the PROVISIONED roster (variable length per wagon). Every
	 * fitted node appears in sens[]; a node not currently heard is reported
	 * with "miss":1 so the cloud sees it as missing rather than silently
	 * absent. This makes the message size scale with the wagon's node count.
	 */
	int64_t now = k_uptime_get();
	for (int i = 0; i < WAGON_NODE_COUNT && len < (int)sizeof(msg)-160; i++) {
		uint8_t id = WAGON_NODES[i].id;
		uint8_t ty = WAGON_NODES[i].type;
		char ser[20]; node_serial(ser, sizeof(ser), ty, id);

		const char *pos = WAGON_NODES[i].pos ? WAGON_NODES[i].pos : "";
		struct sw_node_entry e;
		if (ble_sensors_get(id, &e)) {
			/*
			 * Primary reading for this node type.
			 *
			 * A DOOR node carries its state in the advert FLAGS, not in
			 * `value` - value is ALWAYS 0 there. So the heartbeat used to
			 * report "val":0,"unit":"state" for a door standing wide open,
			 * which a cloud consumer reads as CLOSED - directly
			 * contradicting the DOOR_OPEN event published from that very
			 * same advert. Derive it from the flag: 1 = open, 0 = closed.
			 */
			int val;
			const char *unit;

			if (ty == SW_TYPE_DOOR) {
				val  = (e.data.flags & SW_FLAG_DOOR_OPEN) ? 1 : 0;
				unit = "state";
			} else if (ty == SW_TYPE_BEARING ||
				   ty == SW_TYPE_TANK_TEMP) {
				/* Both send temperature in DECIDEGREES. Uplink
				 * Schema s7 requires typ "btemp" AND "ttemp" to
				 * report degC with unit "C"; the tank was being
				 * published raw (so 23.4 C read as 234) under
				 * unit "state" - a spec breach on both counts. */
				val  = (e.data.value == SW_VAL_NA)
					? 0 : e.data.value / 10;
				unit = "C";
			} else {
				val  = e.data.value;
				unit = "state";
			}

			/*
			 * A sensor that did not answer sends SW_VAL_NA, not a
			 * number. Publishing it raw would put -32768 (or -3276.8
			 * C once scaled) into the cloud as though it were a real
			 * measurement. JSON null is the honest encoding: the
			 * reading is absent, and a consumer cannot mistake it for
			 * data. Both "val" and "t" can be absent independently -
			 * on the tank, the RTD and the STS4x fail separately.
			 */
			char val_s[16], t_s[16];

			if (e.data.value == SW_VAL_NA) {
				snprintf(val_s, sizeof(val_s), "null");
			} else {
				snprintf(val_s, sizeof(val_s), "%d", val);
			}
			/*
			 * "t" is a TEMPERATURE field, so it may only carry value2
			 * when value2 actually holds one - see the contract in
			 * sensor_proto.h. Two cases where it does not:
			 *
			 *   - any node with IMPACT set, where value2 is centi-g
			 *   - the DOOR, which always reports centi-g there and
			 *     puts its internal temperature in "value" instead
			 *
			 * Publishing those as "t" would put a shock magnitude into
			 * a temperature field, which no consumer could detect.
			 */
			if (ty == SW_TYPE_DOOR) {
				if (e.data.value == SW_VAL_NA) {
					snprintf(t_s, sizeof(t_s), "null");
				} else {
					snprintf(t_s, sizeof(t_s), "%d",
						 e.data.value);
				}
			} else if ((e.data.flags & SW_FLAG_IMPACT) ||
				   e.data.value2 == SW_VAL_NA) {
				snprintf(t_s, sizeof(t_s), "null");
			} else {
				snprintf(t_s, sizeof(t_s), "%d", e.data.value2);
			}

			len += snprintf(msg+len, sizeof(msg)-len,
			  "%s{\"node\":\"%s\",\"pos\":\"%s\",\"typ\":\"%s\",\"val\":%s,"
			  "\"unit\":\"%s\",\"t\":%s,\"bat\":%d,\"age\":%lld}",
			  (i?",":""), ser, pos, sw_typ_str(ty),
			  val_s, unit,
			  t_s, e.data.batt,
			  (now - e.ts_ms)/1000);
		} else {
			len += snprintf(msg+len, sizeof(msg)-len,
			  "%s{\"node\":\"%s\",\"pos\":\"%s\",\"typ\":\"%s\",\"miss\":1}",
			  (i?",":""), ser, pos, sw_typ_str(ty));
		}
	}
	/*
	 * Condition bands (RDSO s7.10/s7.11), one entry per BEARING node in roster
	 * order. These used to be the literals ["G"],["G"], so a bearing could
	 * cross into Red, raise a correct BAND_CHANGE event, and still be reported
	 * Green by the very next heartbeat. gwalarm_band() returns the same value
	 * the event was derived from, so the two uplinks now agree.
	 *
	 * "whl" mirrors "brg": the wheel band is derived from the same bearing
	 * node - there is no separate wheel sensor on this wagon.
	 */
	char brg[SW_MAX_NODES * 6 + 1];
	int  bl = 0;
	int  nbrg = 0;

	/*
	 * whl is PER AXLE - four entries for eight bearings (Rev.1 s5.4).
	 *
	 * Eight bearing ends, four wheelsets: each axle carries two bearing
	 * nodes, one at each end, and both of their accelerometers see the same
	 * wheelset. So the axle takes the WORSE of the two wheel bands - a flat
	 * is a property of the wheelset, and either end can detect it.
	 *
	 * Two things were wrong before: whl printed the brg buffer, so wheel
	 * condition came from bearing TEMPERATURE rather than vibration, and it
	 * therefore carried eight entries where the cloud expects four.
	 */
	char whl[8 * 6 + 1];
	int  wl = 0;
	int  nwhl = 0;

	for (int i = 0; i < WAGON_NODE_COUNT && bl < (int)sizeof(brg) - 6; i++) {
		if (WAGON_NODES[i].type != SW_TYPE_BEARING) {
			continue;
		}
		char bd = gwalarm_band(WAGON_NODES[i].id);
		/*
		 * Fold the two bearings of an axle into ONE wheel entry. Ids run
		 * L,R per axle (0/1 = B1-A1, 2/3 = B1-A2, ...), so the axle is
		 * complete on the ODD id and that is where the entry is emitted.
		 */
		if ((WAGON_NODES[i].id & 1u) && wl < (int)sizeof(whl) - 6) {
			char wa = gwalarm_wheel_band(WAGON_NODES[i].id - 1);
			char wb = gwalarm_wheel_band(WAGON_NODES[i].id);
			/*
			 * Worst-of: R beats Y beats G. '?' only when BOTH ends
			 * are unknown, so one silent node cannot mask a flat
			 * that the other end can still see.
			 */
			char wd = (wa == 'R' || wb == 'R') ? 'R'
				: (wa == 'Y' || wb == 'Y') ? 'Y'
				: (wa == 'G' || wb == 'G') ? 'G' : '?';

			if (wd == 'G' || wd == 'Y' || wd == 'R') {
				wl += snprintf(whl + wl, sizeof(whl) - wl,
					       "%s\"%c\"", nwhl ? "," : "", wd);
			} else {
				wl += snprintf(whl + wl, sizeof(whl) - wl,
					       "%snull", nwhl ? "," : "");
			}
			nwhl++;
		}

		/*
		 * Unknown is null, not a letter. s7.11 defines three bands, so a
		 * fourth character here would be an undocumented extension -
		 * whereas null is what this file already uses for val, t, bkp and
		 * the gsm fields when there is no measurement behind them.
		 */
		if (bd == 'G' || bd == 'Y' || bd == 'R') {
			bl += snprintf(brg + bl, sizeof(brg) - bl, "%s\"%c\"",
				       nbrg ? "," : "", bd);
		} else {
			bl += snprintf(brg + bl, sizeof(brg) - bl, "%snull",
				       nbrg ? "," : "");
		}
		nbrg++;
	}
	brg[bl] = '\0';
	whl[wl] = '\0';

	struct charger_status cs2;
	(void)charger_read(&cs2);   /* cached; same sample pwr_compact used */

	uint32_t bufold = 0, bufnew = 0;

	storage_seq_span(&bufold, &bufnew);

	/*
	 * aut - estimated autonomy in hours at the CURRENT draw (Rev.1 s5.3).
	 *
	 * Only meaningful while actually discharging: ibat_ma is negative then,
	 * positive while charging. Reported null when charging, when the draw is
	 * too small to divide by, or when the charger read failed - an autonomy
	 * computed from a charging current would be meaningless, and one invented
	 * for an idle pack worse still.
	 *
	 * Rests on GW_BATT_CAPACITY_MAH, a placeholder until the fitted pack's
	 * part number is confirmed. See app_config.h.
	 */
	char aut_s[16];

	if (cs2.valid && cs2.ibat_ma < -5) {
		uint32_t draw_ma = (uint32_t)(-cs2.ibat_ma);
		uint32_t rem_mah = ((uint32_t)GW_BATT_CAPACITY_MAH * cs2.soc) / 100u;

		snprintf(aut_s, sizeof(aut_s), "%u", rem_mah / draw_ma);
	} else {
		snprintf(aut_s, sizeof(aut_s), "null");
	}
	len += snprintf(msg+len, sizeof(msg)-len,
		"],\"band\":{\"brg\":[%s],\"whl\":[%s]},", brg, whl);
	len += snprintf(msg+len, sizeof(msg)-len,
		/*
		 * Every value here is now measured or explicitly unknown.
		 *
		 * WAS FABRICATED: src "lto", bkp {ok:true, rem:18450}, cell "0",
		 * rssi -71, buf.old/new 0 - all literals in this format string,
		 * none ever sampled. A -71 dBm reading looked like a healthy link
		 * while the modem could not attach at all.
		 *
		 * bkp is now null/null: there is no ADC on the Li-SOCl2 cell and
		 * no coulomb count for it, so its health is genuinely unknown to
		 * this firmware. null says that; 18450 mAh claimed a reserve that
		 * does not exist.
		 *
		 * rssi 0 and cell "" both mean "not measured" - 0 is not a legal
		 * RSSI and "" is not a legal cell id, so neither can be mistaken
		 * for data.
		 */
		"\"pwr2\":{\"src\":\"%s\",\"lto\":{\"v\":%u,\"soc\":%u,"
		"\"chg\":\"%s\"},"
		"\"sol\":{\"vin\":%u,\"i\":%d,\"st\":\"%s\"},"
		"\"bkp\":{\"ok\":null,\"v\":null,\"rem\":null,\"use\":null},"
		"\"aut\":%s},"
		"\"gsm\":{\"cell\":\"%s\",\"rssi\":%d},"
		"\"buf\":{\"n\":%d,\"old\":%u,\"new\":%u},"
		"\"up\":%lld,\"fw\":\"%s\"}}",
		active_src(&cs2),
		cs2.vbat_mv, cs2.soc, charger_chg_str(cs2.chg_state),
		cs2.vbus_mv, cs2.ibat_ma,
		!cs2.vbus_present ? "off" : (cs2.ibat_ma > 0 ? "charging"
							     : "present"),
		aut_s,
		modem_simple_cell(), modem_simple_rssi_dbm(),
		storage_count(), bufold, bufnew,
		k_uptime_get()/3600000, FW_VERSION);

	char t[96]; topic(t, sizeof(t), "hb");
	return telem_send(t, msg);
}

/* ---------------- ALARM (up/alarm, §3) ---------------- */
/*
 * Condition band from a bearing temperature, in decidegrees.
 *
 * Deliberately duplicated from gwalarm.c's band_of() rather than exported:
 * that one reads per-node latched state, while this needs the band of THIS
 * advert, at the instant the alarm was raised. Both use the same s7.10
 * thresholds, so they cannot disagree about where the boundaries are.
 */
static char band_of_temp_c10(int16_t temp_c10)
{
	int c = temp_c10 / 10;

	if (c >= BAND_RED_C)    { return 'R'; }
	if (c >= BAND_YELLOW_C) { return 'Y'; }
	return 'G';
}

int telem_pub_node_alarm(const struct sw_adv *nd, int moving)
{
	static char msg[640];
	struct gnss_fix f; gnss_get(&f);
	char ser[20]; node_serial(ser, sizeof(ser), nd->node_type, nd->node_id);

	/* Base code by node type. An IMPACT-driven alarm is a tamper/shock event,
	 * distinct from the node's primary condition (over-temp / unauth-open), so
	 * relabel it - the advert's IMPACT flag tells them apart.
	 *
	 * The code is the SPEC's "TAMPER" (Protocol Rev.1 s3.2: "Gateway/sensor
	 * removal or tamper"), NOT a per-type invention. This used to emit
	 * DOOR_TAMPER / TANK_TAMPER, which no cloud parser built to the protocol
	 * would recognise - and which silently excluded the other 17 node types.
	 * The node serial already travels in the "node" field, so the receiver can
	 * still tell which device was tampered with. */
	const char *code = sw_alarm_code(nd->node_type, moving);
	if (nd->flags & SW_FLAG_IMPACT) {
		code = "TAMPER";
	}

	/*
	 * Severity follows Rev.1 s3.1, which defines TWO vocabularies:
	 * "yellow | red" for condition-band alarms (bearing/wheel, s7.10) and
	 * "warn | crit" for everything else.
	 *
	 * This was hardcoded "red" for every node alarm, which was wrong both
	 * ways. A bearing in the YELLOW band at 70 C reported the same severity
	 * as one in the RED band at 95 C - collapsing "watch this" and "stop the
	 * train" into one value and defeating the point of graduated bands. And
	 * "red" is not even a legal severity for the non-band alarms (TILT,
	 * DOOR_UNAUTH, HANDBRAKE_MOVING, TANK_OVERTEMP): those take crit.
	 *
	 * A TAMPER relabel is a shock event, not a bearing condition, so it takes
	 * the non-band vocabulary regardless of which node reported it.
	 */
	const char *sev = "crit";

	if (nd->node_type == SW_TYPE_BEARING && !(nd->flags & SW_FLAG_IMPACT)) {
		char b = band_of_temp_c10(nd->value);

		/* Only Y and R can raise an alarm; a Green reading that somehow
		 * arrives with the alarm flag set is treated as red rather than
		 * downgraded, because the node asserted a fault we cannot see. */
		sev = (b == 'Y') ? "yellow" : "red";
	}

	/* Per-type alarm value / unit / threshold ("cg" = centi-g = |a| x100;
	 * "state" = 1 open / 0 closed; thr 0 where not applicable). */
	int val; const char *unit; int thr;
	if (nd->flags & SW_FLAG_IMPACT) {
		/* A TAMPER alarm reports the SHOCK, whatever the node normally
		 * measures. Reporting a bearing's temperature as the value of a
		 * tamper alarm would be meaningless to the operator - and it is what
		 * the old per-type switch did for every type except DOOR and TANK.
		 *
		 * This is only sound because every node type now puts |a| x100 in
		 * value2 whenever IMPACT is set (sensor_proto.h). Before that the
		 * tank published its INTERNAL TEMPERATURE here, labelled "cg". */
		val = nd->value2; unit = "cg"; thr = 0;
	} else {
		switch (nd->node_type) {
		case SW_TYPE_BEARING:
			val = nd->value / 10; unit = "C"; thr = 95; break;
		case SW_TYPE_TANK_TEMP:
			val = nd->value / 10; unit = "C"; thr = 60; break;
		case SW_TYPE_DOOR:
			val = (nd->flags & SW_FLAG_DOOR_OPEN) ? 1 : 0;
			unit = "state"; thr = 0; break;
		default:
			val = nd->value; unit = "state"; thr = 0; break;
		}
	}

	int len = envelope(msg, sizeof(msg), "alarm");
	snprintf(msg+len, sizeof(msg)-len,
		"{\"code\":\"%s\",\"sev\":\"%s\",\"node\":\"%s\",\"pos\":\"%s\","
		"\"val\":%d,\"unit\":\"%s\",\"thr\":%d,\"st\":\"set\","
		"\"aid\":\"al-%u\",\"spd\":%.0f}}",
		code, sev, ser, node_pos(nd->node_id), val, unit, thr,
		g_seq, f.speed_kmh);

	char t[96]; topic(t, sizeof(t), "alarm");
	return telem_send(t, msg);
}

int telem_pub_impact_alarm(double g)
{
	static char msg[512];
	struct gnss_fix f; gnss_get(&f);
	int len = envelope(msg, sizeof(msg), "alarm");
	snprintf(msg+len, sizeof(msg)-len,
		/* Non-band alarm: s3.1 gives these "warn | crit", not the
		 * bearing/wheel band vocabulary. */
		"{\"code\":\"IMPACT\",\"sev\":\"crit\",\"node\":null,\"val\":%.1f,"
		"\"unit\":\"g\",\"thr\":4.0,\"st\":\"set\",\"aid\":\"al-%u\","
		"\"spd\":%.0f}}", g, g_seq, f.speed_kmh);
	char t[96]; topic(t, sizeof(t), "alarm");
	return telem_send(t, msg);
}

/* ---------------- NODE HEALTH (SENSOR_FAULT alarm) ---------------- */
static bool node_down[SW_MAX_NODES];

static int pub_sensor_fault(uint8_t id, uint8_t type, int set)
{
	static char msg[512];
	struct gnss_fix f; gnss_get(&f);
	char ser[20]; node_serial(ser, sizeof(ser), type, id);

	int len = envelope(msg, sizeof(msg), "alarm");
	snprintf(msg+len, sizeof(msg)-len,
		"{\"code\":\"SENSOR_FAULT\",\"sev\":\"warn\",\"node\":\"%s\","
		"\"pos\":\"%s\",\"val\":1,\"unit\":\"state\",\"thr\":%u,\"st\":\"%s\","
		"\"aid\":\"al-%u\",\"spd\":%.0f}}",
		ser, node_pos(id), (unsigned)(NODE_STALE_MS/1000),
		set ? "set" : "clear", g_seq, f.speed_kmh);

	char t[96]; topic(t, sizeof(t), "alarm");
	return telem_send(t, msg);
}

/*
 * Detect nodes that stopped responding. For every PROVISIONED node: if it has
 * not been heard within NODE_STALE_MS (or was never heard once past the boot
 * grace), it is DOWN. Emit SENSOR_FAULT 'set' on the transition to down and
 * 'clear' when it comes back - one alarm per transition, not every heartbeat.
 */
void telem_check_node_health(void)
{
	int64_t now = k_uptime_get();

	/*
	 * BOOT-STORM SUPPRESSION.
	 *
	 * A node that has NEVER been heard used to go "down" the moment uptime
	 * passed NODE_STALE_MS - even though the gateway may not yet have had a
	 * realistic chance to hear it. On a cold start that faults the WHOLE
	 * roster at once: up to 19 SENSOR_FAULT publishes in one cycle, each a
	 * QoS 1 round trip. That burst is what the cellular link fails to carry.
	 *
	 * Give the nodes a fair hearing first: a never-heard node is only
	 * declared down once the gateway has been up for NODE_STALE_MS *plus* a
	 * grace window covering a couple of sub-node advert periods. A node that
	 * HAS been heard is unaffected - a genuine dropout is still caught on the
	 * normal NODE_STALE_MS timer.
	 */
	const int64_t never_heard_after = (int64_t)NODE_STALE_MS +
					  (int64_t)NODE_FIRST_HEARD_GRACE_MS;

	for (int i = 0; i < WAGON_NODE_COUNT; i++) {
		uint8_t id = WAGON_NODES[i].id;
		uint8_t ty = WAGON_NODES[i].type;
		if (id >= SW_MAX_NODES) {
			continue;
		}
		struct sw_node_entry e;
		bool seen = ble_sensors_get(id, &e);
		bool down = seen ? ((now - e.ts_ms) > (int64_t)NODE_STALE_MS)
				 : (now > never_heard_after);

		if (down == node_down[id]) {
			continue;                       /* no state change */
		}

		/*
		 * ONLY LATCH THE EDGE ONCE THE ALARM IS SAFELY HANDED OFF.
		 *
		 * This used to set node_down[id] before publishing, so the edge
		 * was consumed by the ATTEMPT rather than by delivery. When the
		 * link was down the alarm was lost, and the gateway then believed
		 * the cloud had been told and never mentioned the node again: a
		 * permanently missing SENSOR_FAULT.
		 *
		 * telem_send() returns 0 when the record was PUBLISHED or safely
		 * BUFFERED, so latching on 0 is correct with or without the NOR.
		 */
		if (pub_sensor_fault(id, ty, down ? 1 : 0) != 0) {
			/* Link is down: keep the edge and retry next cycle.
			 * Abandon the rest of the sweep too - the remaining
			 * publishes would each burn a prompt timeout for
			 * nothing, and the same retry brings them back. */
			printk("TELEM: node %u fault alarm deferred (link down)\n",
			       id);
			break;
		}
		node_down[id] = down;

		/* PACE the burst: consecutive QoS 1 publishes each need a PUBACK
		 * round trip, and firing the whole roster back to back is what
		 * tips a marginal link over (+QMTPUB ...,1,1 then +QMTSTAT: 0,8).
		 * A short gap costs nothing on a healthy link and keeps a bad one
		 * alive. Only on a real state CHANGE, so a steady wagon with every
		 * node present never pays it. */
		k_msleep(ALARM_BURST_GAP_MS);
	}
}

/* ---------------- EVENT (up/event, §4) ---------------- */
/*
 * `node` is the sensor serial for a NODE-LEVEL event (a specific door, bearing,
 * load cell or handbrake), or NULL for a chassis-level one (BOOT, TRAIN_START,
 * SRC_SWITCH, GEOFENCE_*). Uplink Schema s4 sets the rule: node is null only
 * where the condition belongs to the wagon rather than to one sensor.
 *
 * This used to be hardcoded null for EVERY event, which made a DOOR_OPEN
 * indistinguishable across the four door nodes on a wagon - the cloud was told
 * that a door opened but never which one.
 */
static int pub_event_impl(const char *code, const char *from, const char *to,
			  const char *trip, const char *node)
{
	static char msg[512];
	int len = envelope(msg, sizeof(msg), "event");
	snprintf(msg+len, sizeof(msg)-len,
		"{\"code\":\"%s\",\"from\":%s%s%s,\"to\":%s%s%s,"
		"\"node\":%s%s%s,\"trip\":%s%s%s,\"info\":{}}}",
		code,
		from?"\"":"", from?from:"null", from?"\"":"",
		to?"\"":"",   to?to:"null",     to?"\"":"",
		node?"\"":"", node?node:"null", node?"\"":"",
		trip?"\"":"", trip?trip:"null", trip?"\"":"");
	char t[96]; topic(t, sizeof(t), "event");
	return telem_send(t, msg);
}

int telem_pub_event(const char *code, const char *from,
		    const char *to, const char *trip)
{
	return pub_event_impl(code, from, to, trip, NULL);
}

int telem_pub_event_node(const char *code, const char *from, const char *to,
			 const char *trip, uint8_t node_type, uint8_t node_id)
{
	char ser[20];
	node_serial(ser, sizeof(ser), node_type, node_id);
	return pub_event_impl(code, from, to, trip, ser);
}

/* ---------------- RESPONSE (up/resp, §6.3) ---------------- */
int telem_pub_response(const char *cid, const char *cmd, const char *res,
		       const char *ec, const char *pl_json)
{
	static char msg[640];
	int len = envelope(msg, sizeof(msg), "resp");
	snprintf(msg+len, sizeof(msg)-len,
		"{\"cid\":\"%s\",\"cmd\":\"%s\",\"res\":\"%s\",\"ec\":%s%s%s,"
		"\"pl\":%s}}",
		cid, cmd, res,
		ec?"\"":"", ec?ec:"null", ec?"\"":"",
		pl_json ? pl_json : "{}");
	char t[96]; topic(t, sizeof(t), "resp");
	return telem_send(t, msg);
}

/*
 * Generic alarm publisher (protocol Rev.1 section 3.1).
 *
 * telem_pub_node_alarm() derives everything from a node's type+flags, which
 * covers the relay cases. Gateway-level conditions - FLAT_WHEEL, OVERLOAD,
 * LOW_BATTERY, NODE_LOW_BATTERY, DERAIL - are decided by gwalarm.c from
 * context the node does not have, so they need to state code/sev/val/thr
 * explicitly. `node` may be NULL for gateway-level alarms (section 3.1 says the
 * field is then null, not omitted).
 */
int telem_pub_alarm(const char *code, const char *sev, const char *node_ser,
		    double val, const char *unit, double thr, const char *st)
{
	static char msg[640];
	struct gnss_fix f;

	gnss_get(&f);

	int len = envelope(msg, sizeof(msg), "alarm");

	snprintf(msg + len, sizeof(msg) - len,
		 "{\"code\":\"%s\",\"sev\":\"%s\",\"node\":%s%s%s,"
		 "\"val\":%.1f,\"unit\":\"%s\",\"thr\":%.1f,\"st\":\"%s\","
		 "\"aid\":\"al-%u\",\"spd\":%.0f}}",
		 code, sev,
		 node_ser ? "\"" : "", node_ser ? node_ser : "null",
		 node_ser ? "\"" : "",
		 val, unit, thr, st, g_seq, f.speed_kmh);

	char t[96];

	topic(t, sizeof(t), "alarm");
	return telem_send(t, msg);
}

/* Build the canonical NODE-<type>-<id> serial for a cached node. */
void telem_node_serial(char *o, size_t n, uint8_t type, uint8_t id)
{
	node_serial(o, n, type, id);
}
