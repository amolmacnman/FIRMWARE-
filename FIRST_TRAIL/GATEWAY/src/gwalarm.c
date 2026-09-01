/*
 * Gateway-level alarm/event detection - see gwalarm.h.
 *
 * Every check here is an EDGE detector: it fires when a condition changes, not
 * while it persists. Without that the 10-minute heartbeat would re-publish the
 * same DOOR_OPEN or SRC_SWITCH forever, and the cloud would have to de-duplicate
 * a stream of identical events. The previous state therefore has to be kept
 * across cycles - that is what the s_prev_* arrays are for.
 */
#include <zephyr/kernel.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

/* picolibc does not define M_PI without _GNU_SOURCE/_XOPEN_SOURCE. */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "gwalarm.h"
#include "telemetry.h"
#include "ble_sensors.h"
#include "nodes.h"
#include "charger.h"
#include "power.h"
#include "config.h"
#include "gnss.h"
#include "app_config.h"

/* ---- previous state, for edge detection --------------------------------- */

struct node_prev {
	bool    known;
	bool    door_open;
	bool    hb_applied;
	int16_t load;
	char    band;        /* 'G' | 'Y' | 'R' */
	bool    low_batt;
	bool    tank_hot;    /* TANK_OVERTEMP latch (set/clear edges only) */
};

static struct node_prev s_prev[SW_MAX_NODES];

static bool s_prev_charging;
static bool s_prev_charge_known;
static char s_prev_src[4];       /* "lto" | "bkp" | "trn" */
static bool s_prev_lowbatt;
static bool s_prev_inzone;
static bool s_zone_known;

/* ---- helpers ------------------------------------------------------------ */

static char band_of(int16_t temp_c10)
{
	int c = temp_c10 / 10;

	if (c >= BAND_RED_C)    { return 'R'; }
	if (c >= BAND_YELLOW_C) { return 'Y'; }
	return 'G';
}

char gwalarm_band(uint8_t node_id)
{
	if (node_id >= SW_MAX_NODES || !s_prev[node_id].known) {
		/*
		 * Never heard from, so there is NO measurement to band.
		 *
		 * This used to return 'G'. The argument was that absence is
		 * already reported by "miss":1 and SENSOR_FAULT (s7.14) - true,
		 * but it left one heartbeat saying "no data" and "healthy" about
		 * the same node simultaneously. s7.11 makes the condition band
		 * the headline health indicator, so any dashboard colouring
		 * wagons from band alone showed GREEN for a bearing that was
		 * unfitted, flat or out of range.
		 *
		 * '?' is mapped to JSON null by the caller - the same encoding
		 * this project already uses for every other absent reading.
		 */
		return '?';
	}

	char b = s_prev[node_id].band;

	/* Only bearing nodes get a band assigned, so every other type still holds
	 * the zero-initialised value. Anything unset is unknown, not Green - a
	 * heard node whose band has not been computed has no more basis for a
	 * health claim than one never heard at all. */
	return (b == 'G' || b == 'Y' || b == 'R') ? b : '?';
}

char gwalarm_wheel_band(uint8_t node_id)
{
	struct sw_node_entry e;

	if (node_id >= SW_MAX_NODES || !ble_sensors_get(node_id, &e)) {
		return '?';                    /* never heard - nothing to band */
	}
	if (e.data.node_type != SW_TYPE_BEARING) {
		return '?';                    /* only bearings carry vibration */
	}
	/*
	 * SW_VAL_NA means the node produced no vibration index this cycle.
	 * Banding that Green would assert a healthy wheel from a reading that
	 * does not exist - the same mistake the temperature band used to make
	 * for nodes it had never heard.
	 */
	if (e.data.value2 == SW_VAL_NA) {
		return '?';
	}

	if (e.data.value2 >= FLAT_WHEEL_VIB_THRESH) {
		return 'R';
	}
	if (e.data.value2 >= WHEEL_VIB_YELLOW) {
		return 'Y';
	}
	return 'G';
}

/*
 * Great-circle distance in metres (equirectangular approximation).
 * Accurate to well under a percent at geofence scale (hundreds of metres to a
 * few km) and far cheaper than haversine on a Cortex-M.
 */
static double dist_m(double lat1, double lon1, double lat2, double lon2)
{
	const double R = 6371000.0;
	double dlat = (lat2 - lat1) * (M_PI / 180.0);
	double dlon = (lon2 - lon1) * (M_PI / 180.0);
	double mlat = (lat1 + lat2) * 0.5 * (M_PI / 180.0);
	double x = dlon * cos(mlat);

	return R * sqrt(dlat * dlat + x * x);
}

/* ---- public ------------------------------------------------------------- */

void gwalarm_init(void)
{
	memset(s_prev, 0, sizeof(s_prev));
	s_prev_charge_known = false;
	s_zone_known        = false;
	s_prev_lowbatt      = false;
	strcpy(s_prev_src, "");
}

const char *gwalarm_impact(double g, bool moving)
{
	/*
	 * Section 5.3.3 derailment is IMU + load-sensor fusion. The gateway has the
	 * IMU; a shock above DERAIL_G_THRESH *while the wagon is moving* is the
	 * signature that distinguishes a derailment from shunting or coupling.
	 * A shock while STOPPED is never a derailment - it is yard handling - so
	 * it degrades to IMPACT regardless of magnitude.
	 *
	 * NOTE: this is the IMU half only. Full section 5.3.3 fusion also requires a
	 * corroborating load-sensor discontinuity, which needs load nodes fitted;
	 * with none present this will over-report on severe shunting.
	 */
	bool derail = moving && (g >= DERAIL_G_THRESH);
	const char *code = derail ? "DERAIL" : "IMPACT";

	/* Non-band alarms take s3.1's "warn | crit" vocabulary; "red" belongs to
	 * the bearing/wheel condition bands. */
	telem_pub_alarm(code, "crit", NULL, g, "g",
			derail ? DERAIL_G_THRESH : IMPACT_G_THRESH, "set");
	return code;
}

void gwalarm_link_up(bool was_offline)
{
	if (was_offline) {
		/* Section 4.2: connectivity restored, buffer flush begins. Published
		 * BEFORE the flush so the cloud can attribute the backlog. */
		telem_pub_event("CONN_RESTORED", "offline", "online", NULL);
	}
}

void gwalarm_eval(bool moving)
{
	char ser[24];

	/* ---- per-node conditions ---- */
	for (int i = 0; i < wagon_node_count(); i++) {
		uint8_t id = wagon_node_at(i)->id;
		uint8_t ty = wagon_node_at(i)->type;
		struct sw_node_entry e;

		if (id >= SW_MAX_NODES || !ble_sensors_get(id, &e)) {
			continue;      /* never heard: SENSOR_FAULT covers it */
		}

		struct node_prev *p = &s_prev[id];
		const struct sw_adv *a = &e.data;

		telem_node_serial(ser, sizeof(ser), ty, id);

		/* NODE_LOW_BATTERY - advisory (sev warn), section 3.2. Edge-triggered so a
		 * flat node does not re-alarm every cycle for the rest of its life. */
		bool low = (a->batt <= NODE_LOWBATT_PCT);

		if (low && (!p->known || !p->low_batt)) {
			telem_pub_alarm("NODE_LOW_BATTERY", "warn", ser,
					a->batt, "%", NODE_LOWBATT_PCT, "set");
		} else if (!low && p->known && p->low_batt) {
			telem_pub_alarm("NODE_LOW_BATTERY", "warn", ser,
					a->batt, "%", NODE_LOWBATT_PCT, "clear");
		}
		p->low_batt = low;

		switch (ty) {
		case SW_TYPE_BEARING: {
			/*
			 * FLAT_WHEEL (RDSO s7.13): a wheel flat shows up as a vibration
			 * signature, not as heat - that is what separates it from
			 * HOT_BEARING, which the node itself flags. value2 carries the
			 * vibration index.
			 *
			 * RDSO s7.9 IS EXPLICIT: "Bearing and Wheel Health Vibration
			 * alerts shall be measured and transmitted when the train is
			 * operating in normal service i.e. speed greater than 15 km/h."
			 * Below that the vibration signature is not diagnostic - a
			 * stationary or crawling wagon produces shunting transients that
			 * would read as defects. So the speed gate is a COMPLIANCE
			 * requirement, not an optimisation.
			 */
			struct gnss_fix vf;

			gnss_get(&vf);
			if (vf.speed_kmh > VIBRATION_MIN_KMH &&
			    a->value2 >= FLAT_WHEEL_VIB_THRESH) {
				/* "red" is correct here: FLAT_WHEEL is a
				 * WHEEL CONDITION BAND alarm (s7.13), so it
				 * takes the band vocabulary, and it only fires
				 * at the red vibration threshold. */
				telem_pub_alarm("FLAT_WHEEL", "red", ser,
						a->value2, "idx",
						FLAT_WHEEL_VIB_THRESH, "set");
			}

			/* BAND_CHANGE (section 7.10): Green/Yellow/Red transition. */
			char b = band_of(a->value);

			if (p->known && b != p->band) {
				char from[2] = { p->band, 0 }, to[2] = { b, 0 };

				telem_pub_event_node("BAND_CHANGE", from, to, NULL,
						     a->node_type, a->node_id);
			}
			p->band = b;
			break;
		}

		case SW_TYPE_LOAD_TILT: {
			/* OVERLOAD (section 5.3): load above the configured limit. */
			if (a->value >= OVERLOAD_THRESH) {
				telem_pub_alarm("OVERLOAD", "crit", ser,
						a->value, "%", OVERLOAD_THRESH, "set");
			}
			/* LOAD_CHANGE: loaded/unloaded at a yard. Hysteresis keeps
			 * suspension movement from generating a stream of events. */
			if (p->known && abs(a->value - p->load) >= LOAD_CHANGE_DELTA) {
				telem_pub_event_node("LOAD_CHANGE", NULL, NULL, NULL,
						     a->node_type, a->node_id);
			}
			p->load = a->value;
			break;
		}

		case SW_TYPE_DOOR: {
			/*
			 * DOOR_OPEN / DOOR_CLOSE are the AUTHORISED counterparts of the
			 * DOOR_UNAUTH alarm. main.c raises the alarm when a door opens
			 * while MOVING; the same physical change while stopped is a
			 * normal loading operation and belongs in the event stream.
			 */
			bool open = (a->flags & SW_FLAG_DOOR_OPEN) != 0;

			if (p->known && open != p->door_open && !moving) {
				telem_pub_event_node(open ? "DOOR_OPEN" : "DOOR_CLOSE",
						     p->door_open ? "open" : "closed",
						     open ? "open" : "closed", NULL,
						     a->node_type, a->node_id);
			}
			p->door_open = open;
			break;
		}

		case SW_TYPE_TANK_TEMP: {
			/*
			 * TANK_OVERTEMP (protocol Rev.1 s3.2, clause s5.4.2).
			 *
			 * This case was absent: the switch handled BEARING, LOAD_TILT,
			 * DOOR and HANDBRAKE, so a tank node reporting a cargo
			 * temperature breach fell through and NO TANK_OVERTEMP alarm was
			 * ever raised - the one alarm code in the spec with no
			 * implementation.
			 *
			 * The node itself owns the threshold decision: tank main.c sets
			 * SW_FLAG_ALARM when cargo_x10 >= sn_threshold(), and clears it
			 * only below (threshold - hysteresis), so the flag is already
			 * debounced. The gateway relays that as the spec's alarm rather
			 * than re-deriving it, exactly as HOT_BEARING and
			 * HANDBRAKE_MOVING are relayed from the node's flag.
			 *
			 * IMPACT is deliberately NOT treated as over-temp here: the tank
			 * node ORs impact into SW_FLAG_ALARM too, so the flag alone is
			 * ambiguous. Requiring IMPACT to be absent keeps a tamper shock
			 * from being reported as a cargo temperature breach - that shock
			 * is already covered by the IMPACT/TAMPER path.
			 *
			 * sev "red": a cargo temperature breach is safety-critical.
			 * Uplink Schema s4 permits only "red" or "warn".
			 */
			bool hot = (a->flags & SW_FLAG_ALARM) != 0 &&
				   (a->flags & SW_FLAG_IMPACT) == 0;

			if (hot && (!p->known || !p->tank_hot)) {
				telem_pub_alarm("TANK_OVERTEMP", "crit", ser,
						a->value / 10.0, "C",
						TANK_OVERTEMP_C, "set");
			} else if (!hot && p->known && p->tank_hot) {
				telem_pub_alarm("TANK_OVERTEMP", "crit", ser,
						a->value / 10.0, "C",
						TANK_OVERTEMP_C, "clear");
			}
			p->tank_hot = hot;
			break;
		}

		case SW_TYPE_HANDBRAKE: {
			/* Section 5.6: applied/released while STATIONARY is normal. Applied
			 * while moving is the HANDBRAKE_MOVING alarm, raised on the
			 * node's own ALARM flag - not here. */
			bool applied = (a->value != 0);

			if (p->known && applied != p->hb_applied && !moving) {
				telem_pub_event_node(applied ? "HANDBRAKE_APPLIED"
								   : "HANDBRAKE_RELEASED",
						     p->hb_applied ? "applied" : "released",
						     applied ? "applied" : "released", NULL,
						     a->node_type, a->node_id);
			}
			p->hb_applied = applied;
			break;
		}

		default:
			break;
		}
		p->known = true;
	}

	/* ---- power: CHARGE_START/_STOP, SRC_SWITCH, LOW_BATTERY ---- */
	struct charger_status cs;

	if (charger_read(&cs) == 0 && cs.valid) {
		/*
		 * Section 4.2 CHARGE_START/_STOP. Solar can be *present* but below the
		 * charging threshold in low light or cold (section 2.2), so charging is
		 * defined by actual current INTO the pack, not by VBUS presence.
		 */
		bool charging = cs.vbus_present && (cs.ibat_ma > CHARGE_I_THRESH_MA);

		if (s_prev_charge_known && charging != s_prev_charging) {
			telem_pub_event(charging ? "CHARGE_START" : "CHARGE_STOP",
					s_prev_charging ? "charging" : "off",
					charging ? "charging" : "off", NULL);
		}
		s_prev_charging     = charging;
		s_prev_charge_known = true;

		/*
		 * SRC_SWITCH: which side of the power mux is live.
		 *
		 * Read from the TPS2116 ST pin (Rev.1 s2.2), not inferred. The old
		 * SoC estimate fabricated switchover events: a mis-calibrated
		 * battery map made it report "bkp" while the mux had never moved,
		 * so the cloud saw a power failover that did not happen.
		 */
		bool mux_known = false;
		bool mux_lto = power_mux_is_lto(&mux_known);
		const char *src = mux_known
				? (mux_lto ? "lto" : "bkp")
				: (cs.vbus_present ? "lto"
				   : (cs.vbat_mv <= LTO_SWITCHOVER_MV
					      ? "bkp" : "lto"));

		if (s_prev_src[0] && strcmp(src, s_prev_src) != 0) {
			telem_pub_event("SRC_SWITCH", s_prev_src, src, NULL);
		}
		strncpy(s_prev_src, src, sizeof(s_prev_src) - 1);
		s_prev_src[sizeof(s_prev_src) - 1] = '\0';

		/* LOW_BATTERY: the GATEWAY's own pack (node = null, section 3.2). */
		bool lowb = (cs.soc <= GW_LOWBATT_PCT);

		if (lowb && !s_prev_lowbatt) {
			telem_pub_alarm("LOW_BATTERY", "crit", NULL,
					cs.soc, "%", GW_LOWBATT_PCT, "set");
		} else if (!lowb && s_prev_lowbatt) {
			telem_pub_alarm("LOW_BATTERY", "crit", NULL,
					cs.soc, "%", GW_LOWBATT_PCT, "clear");
		}
		s_prev_lowbatt = lowb;
	}

	/* ---- GEOFENCE_ENTER / _EXIT ---- */
	if (GEOFENCE_RADIUS_M > 0) {
		struct gnss_fix f;

		gnss_get(&f);
		if (f.valid) {
			double d = dist_m(f.lat_deg, f.lon_deg,
					  GEOFENCE_LAT, GEOFENCE_LON);
			bool inzone = (d <= GEOFENCE_RADIUS_M);

			if (s_zone_known && inzone != s_prev_inzone) {
				telem_pub_event(inzone ? "GEOFENCE_ENTER"
						       : "GEOFENCE_EXIT",
						s_prev_inzone ? "in" : "out",
						inzone ? "in" : "out", NULL);
			}
			s_prev_inzone = inzone;
			s_zone_known  = true;
		}
	}
}
