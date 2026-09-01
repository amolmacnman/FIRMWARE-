/*
 * Smart Wagon GATEWAY - Zephyr, nRF54L15.
 * Implements the SmartWagon Telemetry Protocol (Rev.1) Class-A wake cycle:
 * sleep -> wake (schedule / impact / node-alarm / motion) -> publish uplink ->
 * drain queued commands (incl OTA) -> respond -> sleep. Multi-wagon isolation
 * via the BLE group filter and per-wagon MQTT topics.
 *
 * Reuse gnss.c/gnss.h and ec200.c/ec200.h from wagen1 (with the ec200 edits
 * in this folder for clean-session + dn/cmd subscribe + command polling).
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "app_config.h"
#include "gnss.h"
#include "ec200.h"
#include "modem_simple.h"
#include "ble_sensors.h"
#include "telemetry.h"
#include "thermal.h"
#include "ota.h"
#include "nodeota.h"
#include "sensor_proto.h"
#include "modem_cmd.h"
#include "power.h"
#include "watchdog.h"

#include "bma400.h"
#include "motion.h"
#include "charger.h"
#include "storage.h"
#include "config.h"
#include "sw_ids.h"
#include "sw_secure.h"
#include "node_link.h"
#include "gwalarm.h"
#include "nodes.h"

/*
 * Worst-case duration of one report cycle: GNSS fix (up to 120 s at boot,
 * gnss_timeout_s after) + modem attach and PDP activation + the MQTT publishes.
 * Declared to the watchdog via gw_wdt_idle_begin() so a long-but-normal cycle
 * is not mistaken for a hang. feed_fn() tolerates this budget PLUS
 * GW_WDT_IDLE_MARGIN_MS before it stops feeding, so a genuine wedge is still
 * caught - just not at the 120 s progress window, which a report legitimately
 * exceeds.
 */
#define GW_REPORT_BUDGET_MS   (5UL * 60UL * 1000UL)   /* 5 min */

#define EV_SCHED   BIT(0)
#define EV_IMPACT  BIT(1)
#define EV_NALARM  BIT(2)
#define EV_MOTION  BIT(3)   /* debounced motion CONFIRMED a state change  */
#define EV_MPOLL   BIT(4)   /* time to re-sample the BMA400 for debounce  */
static struct k_event ev;

/* Pending-alarm ring (SPSC): producer = on_node_alarm (BLE RX thread), consumer
 * = the main report loop. Replaces a single last_alarm slot so two nodes that
 * alarm in the same instant don't clobber each other (and no torn reads). A
 * full ring drops the newest - persistent alarms simply re-advertise. */
#define ALARM_RING 8
static struct sw_adv      alarm_ring[ALARM_RING];
static uint8_t            ar_head, ar_tail;   /* head=next write, tail=next read */
static struct k_spinlock  ar_lock;
static uint32_t       trip_ctr;
static double         impact_g;     /* magnitude latched at an impact wake */

static void alarm_push(const struct sw_adv *a)
{
	k_spinlock_key_t key = k_spin_lock(&ar_lock);
	uint8_t next = (uint8_t)((ar_head + 1) % ALARM_RING);
	if (next != ar_tail) {                 /* space available */
		alarm_ring[ar_head] = *a;
		ar_head = next;
	}                                      /* else full -> drop newest */
	k_spin_unlock(&ar_lock, key);
}

static bool alarm_pop(struct sw_adv *out)
{
	bool got = false;
	k_spinlock_key_t key = k_spin_lock(&ar_lock);
	if (ar_tail != ar_head) {
		*out = alarm_ring[ar_tail];
		ar_tail = (uint8_t)((ar_tail + 1) % ALARM_RING);
		got = true;
	}
	k_spin_unlock(&ar_lock, key);
	return got;
}

/* ---- schedule timer (RTC stand-in): cadence from the DEBOUNCED state ---- */
static void sched_expiry(struct k_timer *t){ ARG_UNUSED(t); k_event_post(&ev, EV_SCHED); }
K_TIMER_DEFINE(sched_timer, sched_expiry, NULL);
static void arm_schedule(void)
{
	/* cadence from the persisted, server-settable config (working copy) */
	uint32_t ms = (motion_is_moving() ? g_cfg.moving_s : g_cfg.idle_s) * 1000u;
	k_timer_start(&sched_timer, K_MSEC(ms), K_NO_WAIT);
	gw_wdt_idle_begin(ms);   /* watchdog tolerates exactly this sleep */
}

/* ---- motion poll timer: while a transition is pending (or moving) we
 * re-sample the always-on BMA400 every MOTION_POLL_MS to time the debounce.
 * It costs no radio power - only an I2C burst on a rail that is always on. */
static void mpoll_expiry(struct k_timer *t){ ARG_UNUSED(t); k_event_post(&ev, EV_MPOLL); }
K_TIMER_DEFINE(motion_poll_timer, mpoll_expiry, NULL);
static void motion_poll_start(void)
{ k_timer_start(&motion_poll_timer, K_MSEC(MOTION_POLL_MS), K_MSEC(MOTION_POLL_MS)); }

/* ---- callbacks ---- */
/* Per-door latch so one open-while-moving episode raises ONE DOOR_UNAUTH, not a
 * flood (a door re-advertises "open" every second). Re-armed when the wagon
 * stops or that door is next seen closed (see doors_reeval). */
static bool door_latched[SW_MAX_NODES];

/*
 * Per-bearing latch for a vibration escalation, same idea as door_latched: a
 * bearing over its hint limit re-advertises every second, and every one of
 * those must not start a report cycle.
 */
static bool vib_latched[SW_MAX_NODES];

static void on_node_alarm(const struct sw_adv *a, int8_t rssi)
{
	ARG_UNUSED(rssi);
	/* Subnode already flagged ALARM (impact / tamper / over-temp): forward. */
	if (a->flags & SW_FLAG_ALARM) {
		alarm_push(a);
		k_event_post(&ev, EV_NALARM);
		return;
	}
	/* Door-open with no subnode alarm: the GATEWAY decides. Raise DOOR_UNAUTH
	 * only while the train is actually MOVING (authoritative debounced motion),
	 * and only once per open-in-transit episode. A door open at a stop is left
	 * as state in the heartbeat, not an alarm. */
	if (a->node_type == SW_TYPE_DOOR && (a->flags & SW_FLAG_DOOR_OPEN) &&
	    a->node_id < SW_MAX_NODES) {
		if (motion_is_moving() && !door_latched[a->node_id]) {
			door_latched[a->node_id] = true;
			alarm_push(a);
			k_event_post(&ev, EV_NALARM);
		}
	}

	/*
	 * Bearing vibration hint. The node has crossed its OWN limit and dropped
	 * to 1 s advertising so fresh data is available; the FLAT_WHEEL decision
	 * is still ours, because only we know the speed (s7.9 needs > 15 km/h).
	 *
	 * This wake is what makes the node's escalation worth its power. Without
	 * it the node paid ~1800 uA to shout every second while the gateway
	 * ignored it until the next scheduled report - up to 10 minutes later,
	 * which is not the "near real time alert" s7.3 asks for.
	 *
	 * Nothing is pushed to the alarm ring: that path publishes the node's
	 * TYPE code (HOT_BEARING), and this is not an over-temperature. Waking is
	 * enough - the report path takes a fresh fix and gwalarm_eval() then
	 * raises FLAT_WHEEL if, and only if, the speed gate passes.
	 */
	if (a->node_type == SW_TYPE_BEARING && (a->flags & SW_FLAG_VIB_HIGH) &&
	    a->node_id < SW_MAX_NODES) {
		if (motion_is_moving() && !vib_latched[a->node_id]) {
			vib_latched[a->node_id] = true;
			k_event_post(&ev, EV_NALARM);
		}
	}
}

/* Re-arm door latches: fully when the wagon is stopped (no open-in-transit is
 * possible), else per-door once that door is next observed closed. Called on a
 * confirmed motion change and at each heartbeat. */
static void doors_reeval(void)
{
	if (!motion_is_moving()) {
		memset(door_latched, 0, sizeof(door_latched));
		memset(vib_latched, 0, sizeof(vib_latched));
		return;
	}
	for (int id = 0; id < SW_MAX_NODES; id++) {
		struct sw_node_entry e;

		if (door_latched[id] && ble_sensors_get((uint8_t)id, &e) &&
		    e.data.node_type == SW_TYPE_DOOR &&
		    !(e.data.flags & SW_FLAG_DOOR_OPEN)) {
			door_latched[id] = false;   /* closed again -> re-arm */
		}
		/* Same for a bearing whose vibration has settled back under its
		 * hint limit, so a later re-escalation wakes us again. */
		if (vib_latched[id] && ble_sensors_get((uint8_t)id, &e) &&
		    e.data.node_type == SW_TYPE_BEARING &&
		    !(e.data.flags & SW_FLAG_VIB_HIGH)) {
			vib_latched[id] = false;
		}
	}
}

/* Fired ONLY when the debounce confirms a real start/stop (not on blips). */
static void on_motion_confirmed(bool moving){ ARG_UNUSED(moving); k_event_post(&ev, EV_MOTION); }

/* ---- impact wake: BMA400 INT on P1.16 (schematic net BMA_INT) ---- */
#if DT_NODE_EXISTS(DT_ALIAS(bma_int))
static const struct gpio_dt_spec impact = GPIO_DT_SPEC_GET(DT_ALIAS(bma_int), gpios);
static struct gpio_callback impact_cb;
static void impact_isr(const struct device *d, struct gpio_callback *c, uint32_t p)
{ ARG_UNUSED(d); ARG_UNUSED(c); ARG_UNUSED(p); k_event_post(&ev, EV_IMPACT); }
static void impact_init(void){
	if(!gpio_is_ready_dt(&impact)) return;
	gpio_pin_configure_dt(&impact, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&impact, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&impact_cb, impact_isr, BIT(impact.pin));
	gpio_add_callback(impact.port, &impact_cb);
}
#else
static void impact_init(void){}
#endif

/* ---- tiny JSON reader ----------------------------------------------------
 *
 * Locate the VALUE of "key", returning a pointer just past the colon with any
 * whitespace skipped, or NULL if the key is absent.
 *
 * WHITESPACE: these used to build a literal pattern - json_str() searched for
 * "key":" with the quote welded to the colon - so a perfectly valid command
 * typed with a space after the colon ({ "cid": "i01" ... }, which is what every
 * MQTT client's JSON editor produces) simply did not match. cid and cmd came
 * back empty and the gateway answered err/UNSUPPORTED to a command it had in
 * fact received intact. JSON allows whitespace around the colon; so do we now,
 * on both sides of it.
 *
 * KEY-vs-VALUE: searching for the bare token "cmd" is not enough, because our
 * OWN envelope contains "mt": "cmd" - the word appears as a VALUE before it
 * appears as a key. So skip any occurrence that is not followed by a colon and
 * keep looking. (The old welded pattern was accidentally immune to this; a
 * naive relaxation is not.)
 */
static const char *json_find(const char *j, const char *key)
{
	char pat[40];
	snprintf(pat, sizeof(pat), "\"%s\"", key);
	size_t plen = strlen(pat);

	for (const char *p = strstr(j, pat); p; p = strstr(p + 1, pat)) {
		const char *q = p + plen;
		while (*q == ' ' || *q == '\t' || *q == '\r' || *q == '\n') q++;
		if (*q != ':') {
			continue;          /* that was a value, not a key */
		}
		q++;
		while (*q == ' ' || *q == '\t' || *q == '\r' || *q == '\n') q++;
		return q;
	}
	return NULL;
}

static bool json_str(const char *j, const char *key, char *out, size_t n)
{
	const char *p = json_find(j, key);
	if (!p || *p != '"') return false;
	p++;
	size_t i = 0;
	while (*p && *p != '"' && i < n-1) out[i++] = *p++;
	out[i] = '\0';
	return true;
}
static long json_num(const char *j, const char *key)
{
	const char *p = json_find(j, key);
	return p ? strtol(p, NULL, 10) : 0;
}
static bool json_has(const char *j, const char *key)
{
	return json_find(j, key) != NULL;
}
static double json_dbl(const char *j, const char *key)
{
	const char *p = json_find(j, key);
	return p ? strtod(p, NULL) : 0.0;
}
static bool json_bool(const char *j, const char *key, bool dflt)
{
	const char *p = json_find(j, key);
	return p ? (strncmp(p, "true", 4) == 0) : dflt;
}

/*
 * Deterministic per-wagon delay for a GROUP or BULK ota_start.
 *
 * A broadcast reaching a whole rake has every gateway open an HTTP download of
 * the same image in the same second. Spreading them is not optional at fleet
 * scale, but a RANDOM delay would let two wagons still collide and would make
 * a roll-out impossible to predict or reproduce. Deriving the offset from the
 * wagon number instead gives a fixed, evenly-distributed slot per wagon: the
 * same wagon always takes the same slot, and the operator can compute exactly
 * when any given wagon will start.
 */
static uint32_t ota_stagger_ms(void)
{
	uint16_t h = sw_wgn_group(cfg_wagon());   /* CRC16 of the wagon number */

	return (uint32_t)((uint32_t)h % OTA_FLEET_STAGGER_MS);
}

/*
 * Queue a sub-node threshold to one node, all nodes, or all nodes of one type.
 *
 * Returns the number of nodes the command was queued for. Only nodes that have
 * actually been HEARD can be targeted: the gateway needs the node type to seal
 * the BLE frame (it is authenticated in the AAD), and that only comes from an
 * advert. A never-heard node is skipped rather than guessed at.
 */
/*
 * Map the MQTT "param" string to a sub-node downlink command id.
 *
 * This is the SENSOR axis. It composes with the NODE axis (node / type / all)
 * and the FLEET axis ("scope", fanned out per wagon by the cloud), so any
 * combination is expressible: one sensor on one node, one sensor on every node
 * of a type, or one sensor on every node of every wagon.
 *
 * Omitted or "primary" keeps the original behaviour - the node's own
 * type-specific threshold (bearing temp, tank temp, load %, door/handbrake
 * state). Returns 0 for an unrecognised param.
 */
/*
 * Gauge calibration selector for a param name, or -1 if it is not one.
 *
 * Separate from dn_cmd_for_param() because these seven share a single opcode
 * and differ only in the frame's rsvd byte - so the caller needs both halves,
 * and folding the selector into the opcode lookup would mean returning two
 * values from a function whose whole job is to return one.
 *
 * The names mirror the app_config.h constants they replace, so an operator
 * reading a PPK2 trace against that file can see which is which.
 */
static int dn_battcal_sel(const char *param)
{
	if (!strcmp(param, "batt_i_sleep_ua"))     return SW_BATTCAL_I_SLEEP;
	if (!strcmp(param, "batt_i_selfdisch_ua")) return SW_BATTCAL_I_SELFDISCH;
	if (!strcmp(param, "batt_i_meas_ua"))      return SW_BATTCAL_I_MEAS;
	if (!strcmp(param, "batt_t_meas_ms"))      return SW_BATTCAL_T_MEAS;
	if (!strcmp(param, "batt_i_tx_ua"))        return SW_BATTCAL_I_TX;
	if (!strcmp(param, "batt_i_cfgwin_ua"))    return SW_BATTCAL_I_CFGWIN;
	if (!strcmp(param, "batt_usable_mah"))     return SW_BATTCAL_USABLE_MAH;
	return -1;
}

static uint8_t dn_cmd_for_param(const char *param)
{
	if (param[0] == 0 || !strcmp(param, "primary")) {
		return SW_DN_SET_THRESHOLD;
	}
	/* Temperature nodes: the primary sensor IS the temperature, so these
	 * are aliases rather than a separate threshold. */
	if (!strcmp(param, "bearing_temp_c") || !strcmp(param, "tank_temp_c") ||
	    !strcmp(param, "temp_c")) {
		return SW_DN_SET_THRESHOLD;
	}
	if (!strcmp(param, "impact_g")) {
		return SW_DN_SET_IMPACT;      /* BMA400 tamper, milli-g */
	}
	if (!strcmp(param, "vib_index")) {
		return SW_DN_SET_VIB;         /* bearing vibration hint */
	}
	/*
	 * Advertising cadence, in SECONDS. "value" carries the quiet period and
	 * "hyst" the alarm period - the same two wire fields every other param
	 * uses, so no new plumbing is needed and the node / type / all fan-out
	 * below applies to it unchanged.
	 *
	 * The NODE clamps both to 1..3600 s and refuses to let the alarm period
	 * exceed the quiet one, so a careless command can neither flatten a cell
	 * nor invert the point of the fast cadence.
	 */
	if (!strcmp(param, "cadence")) {
		return SW_DN_SET_CADENCE;
	}
	/*
	 * Sub-node battery voltage references, millivolts. "value" is the FRESH
	 * reference and "hyst" the END-OF-LIFE knee - the same two wire fields
	 * every other param uses, so the node / type / all fan-out applies
	 * unchanged.
	 *
	 * These are NOT the gateway's own full/empty pair. A sub-node runs on a
	 * Li-SOCl2 primary cell whose voltage is flat for ~95 % of its life, so
	 * there is no voltage->percent map to set: its SoC is coulomb-counted,
	 * and these two only gate the replacement detector and the end-of-life
	 * backstop. The gateway's rechargeable LTO pack is a different chemistry
	 * with a different question, answered by dn/cmd set_batt.
	 */
	if (!strcmp(param, "batt_mv")) {
		return SW_DN_SET_BATT;
	}
	/*
	 * Coulomb-gauge calibration - seven params, one opcode, distinguished by
	 * the selector dn_battcal_sel() returns for the same name.
	 */
	if (dn_battcal_sel(param) >= 0) {
		return SW_DN_SET_BATTCAL;
	}
	return 0;
}

/*
 * Queue a threshold to every node in scope.
 *
 * Returns how many were queued. `rejected`, if given, receives how many were
 * skipped because the VALUE cannot be reached on that node - which the caller
 * needs to tell "no such node" apart from "impossible value". In a bare count
 * those two are indistinguishable, and they send an operator to completely
 * different places: one to check power and antennas, the other to re-read the
 * units on the value they just sent.
 */
static int queue_threshold_scope(int want_id, int want_type, bool all,
				 uint8_t cmd, uint8_t sel, int16_t value,
				 int16_t hyst, int *rejected)
{
	int n = 0, bad = 0;

	for (int i = 0; i < wagon_node_count(); i++) {
		uint8_t id = wagon_node_at(i)->id;

		if (id >= SW_MAX_NODES) {
			continue;
		}
		if (!all && want_id >= 0 && id != (uint8_t)want_id) {
			continue;
		}

		struct sw_node_entry ne;
		if (!ble_sensors_get(id, &ne)) {
			continue;              /* never heard - cannot address it */
		}
		uint8_t ty = ne.data.node_type;

		if (ty == 0) {
			continue;
		}
		if (want_type >= 0 && ty != (uint8_t)want_type) {
			continue;
		}

		/*
		 * Check reachability HERE as well as on the node.
		 *
		 * The node also refuses an impossible threshold, but its only
		 * way to say so is a GATT error during a config window - by
		 * which time this gateway has already answered the broker
		 * "queued" and the operator has moved on. Checking before the
		 * frame is sealed is the only point where a refusal can still
		 * become an MQTT error the sender actually sees.
		 *
		 * Per node, not per command: a "type" or "all" scope covers
		 * several node types at once, and a value that is nonsense for a
		 * door is fine for a bearing. Nodes that cannot take it are
		 * skipped and the count returned to the caller falls short,
		 * which is what turns into the response.
		 */
		if (cmd == SW_DN_SET_THRESHOLD &&
		    !sw_threshold_sane(ty, value, hyst)) {
			printk("GW: node %u (type %u) cannot reach threshold "
			       "%d/%d - skipped\n", id, ty, value, hyst);
			bad++;
			continue;
		}
		if (cmd == SW_DN_SET_BATTCAL && !sw_battcal_sane(sel, value)) {
			printk("GW: node %u battcal[%u] = %d out of range - "
			       "skipped\n", id, sel, value);
			bad++;
			continue;
		}
		if (cmd == SW_DN_SET_BATT && !sw_batt_mv_sane(value, hyst)) {
			printk("GW: node %u batt refs %d/%d mV out of range - "
			       "skipped\n", id, value, hyst);
			bad++;
			continue;
		}
		if (cmd == SW_DN_SET_IMPACT && !sw_impact_sane(value)) {
			printk("GW: node %u impact %d mg out of range - "
			       "skipped\n", id, value);
			bad++;
			continue;
		}
		if (node_link_queue_cmd_sel(id, ty, cmd, sel, value, hyst) == 0) {
			n++;
		}
	}
	if (rejected) {
		*rejected = bad;
	}
	return n;
}

/* ---- handle one downlink command, publish its response ---- */
static void handle_command(const char *j)
{
	char cid[32] = "", cmd[24] = "";
	json_str(j, "cid", cid, sizeof(cid));
	json_str(j, "cmd", cmd, sizeof(cmd));

	/*
	 * Scope of this command, declared by the sender.
	 *
	 * The gateway subscribes to exactly ONE topic - its own .../dn/cmd (see
	 * the single AT+QMTSUB in ec200.c). A group or bulk roll-out is therefore
	 * fanned out BY THE CLOUD, one publish per wagon, and each copy carries
	 * "scope" so the wagon knows it is part of a fleet action.
	 *
	 * Declaring it in the payload rather than inferring it from the topic is
	 * also forced by the transport: the +QMTRECV URC is parsed for its PAYLOAD
	 * only, so the topic a message arrived on is not available here.
	 *
	 * Scope matters for exactly one thing: whether an ota_start is staggered.
	 * Everything else behaves identically however it was addressed.
	 *
	 * Absent or "individual" -> act immediately, as before.
	 */
	char scope[12] = "";
	json_str(j, "scope", scope, sizeof(scope));
	bool fleet = (!strcmp(scope, "group") || !strcmp(scope, "all") ||
		      !strcmp(scope, "bulk"));

	/*
	 * FITMENT GUARD for fleet-scoped firmware.
	 *
	 * The wagon number now lives in FRAM, so one image can serve many
	 * wagons - but only wagons carrying the SAME SENSORS. WAGON_NODES is
	 * still compiled in, and a gateway running another fitment's image
	 * expects nodes that were never installed: it reports them missing and
	 * raises SENSOR_FAULT forever, while ignoring nodes it does have.
	 *
	 * So a group or bulk ota_start may carry "fitment": the id the image
	 * was built for. If it does not match ours, refuse - and say so, rather
	 * than downloading 290 KB over cellular to install a mismatch. The
	 * field is OPTIONAL: an operator addressing one wagon deliberately can
	 * omit it, and nothing changes for individual updates.
	 *
	 * get_status reports "fitment", so the back office can group wagons
	 * without keeping its own fitment register in step with ours.
	 */
	if (fleet && json_has(j, "fitment")) {
		uint32_t want = (uint32_t)json_num(j, "fitment");
		uint32_t have = wagon_fitment_id();

		if (want != have) {
			char pl[96];

			snprintf(pl, sizeof(pl),
				 "{\"fitment\":%u,\"image_fitment\":%u}",
				 have, want);
			printk("GW: refusing %s - image is for fitment 0x%05X, "
			       "this wagon is 0x%05X\n", cmd, want, have);
			telem_pub_response(cid, cmd, "err", "FITMENT_MISMATCH",
					   pl);
			return;
		}
	}

	printk("GW: cmd '%s' cid=%s scope=%s\n", cmd, cid,
	       scope[0] ? scope : "individual");

	if (!strcmp(cmd, "ota_start")) {
		/*
		 * Routes to the GATEWAY image, or to ONE sub-node.
		 *
		 * This comment previously said sub-node update was "deliberately not
		 * supported" and that any "node" field was rejected. That stopped
		 * being true when nodeota.c and the node_link image transfer landed,
		 * and the code immediately below has supported single-node updates
		 * ever since - so the comment described the opposite of the branch it
		 * introduced.
		 *
		 * What is actually rejected is "type" and "all": NODE_ID is compiled
		 * into each sub-node binary, so one image cannot serve several nodes.
		 * See the BAD_PARAM branch below for the full reasoning.
		 */
		char url[160]="", ver[24]="";
		json_str(j, "url", url, sizeof(url));
		json_str(j, "ver", ver, sizeof(ver));
		int size = json_num(j, "size");

		if (json_has(j, "node") || json_has(j, "type") ||
		    json_bool(j, "all", false)) {
			/*
			 * SUB-NODE image. Staged into the NOR "ota" partition now,
			 * while the modem is up, and delivered to the node later
			 * across as many of its config windows as it takes - the
			 * two links are never up together.
			 *
			 * A FASTWIN is queued alongside so the node opens windows
			 * every few seconds instead of every 10 minutes; without it
			 * a 150 KB image would take most of a day. cfgwin deadlines
			 * fast mode itself, so an abandoned campaign cannot strand
			 * the node at high duty.
			 */
			/*
			 * TARGETS: one node, or every node of a TYPE.
			 *
			 * "type" became possible only when NODE_ID moved out of
			 * the sub-node image into NVS. While it was compiled in,
			 * one image sent to eight bearings would have left all
			 * eight claiming the same id, and the gateway could not
			 * have told which bearing it was hearing from - which is
			 * why this branch used to refuse anything but one node.
			 *
			 * The image is fetched over cellular ONCE and then
			 * streamed to each target over BLE. That asymmetry is the
			 * point: the modem leg is metered and slow, the radio leg
			 * is neither, so eight bearings cost one download.
			 *
			 * Targets come from the ROSTER, not from what happens to
			 * be audible at this instant. A node asleep right now is
			 * still fitted and still needs the update - it is reached
			 * when it next opens a window.
			 */
			uint32_t targets = 0;
			uint8_t  nty     = 0;

			if (json_bool(j, "all", false)) {
				telem_pub_response(cid, cmd, "err", "BAD_PARAM",
					"{\"info\":\"ota_start takes 'node' or "
					"'type' - 'all' would send one type's "
					"image to every sensor on the wagon\"}");
				return;
			}

			if (json_has(j, "type")) {
				nty = (uint8_t)json_dbl(j, "type");
				for (int i = 0; i < wagon_node_count(); i++) {
					const struct wagon_node *w =
						wagon_node_at(i);

					if (w->type == nty &&
					    w->id < SW_MAX_NODES) {
						targets |= (uint32_t)1u << w->id;
					}
				}
				if (targets == 0) {
					telem_pub_response(cid, cmd, "err",
						"NOT_FOUND",
						"{\"info\":\"no node of that type "
						"in this wagon's roster\"}");
					return;
				}
			} else {
				int nid = (int)json_dbl(j, "node");
				struct sw_node_entry ne;

				if (nid < 0 || nid >= SW_MAX_NODES ||
				    !ble_sensors_get((uint8_t)nid, &ne)) {
					telem_pub_response(cid, cmd, "err",
						"NOT_FOUND",
						"{\"info\":\"node not heard\"}");
					return;
				}
				nty     = ne.data.node_type;
				targets = (uint32_t)1u << nid;
			}

			{
				int rc = nodeota_start(url, ver, size,
						       targets, nty);
				char pl[160];

				nodeota_status_json(pl, sizeof(pl));
				if (rc == 0) {
					/*
					 * Fast windows for EVERY target, not
					 * just the first. They are delivered one
					 * after another, and a node still at the
					 * 10-minute cadence when its turn comes
					 * would stall the queue behind it.
					 */
					for (uint8_t id = 0; id < SW_MAX_NODES;
					     id++) {
						if (targets &
						    ((uint32_t)1u << id)) {
							node_link_queue_cmd(id,
								nty,
								SW_DN_FASTWIN,
								NODE_OTA_FASTWIN_S,
								0);
						}
					}
					telem_pub_response(cid, cmd, "ok", NULL, pl);
				} else {
					telem_pub_response(cid, cmd, "err",
							   rc == -EBUSY ? "BUSY" :
							   "BAD_PARAM", pl);
				}
			}
		} else {
			/*
			 * Fleet-scoped update: answer FIRST, then wait out this
			 * wagon's stagger slot before downloading. Answering
			 * first tells the back office the command was accepted
			 * even though the image will not arrive for minutes, so
			 * a roll-out can be tracked as it propagates instead of
			 * looking like a fleet of silent wagons.
			 */
			if (fleet && strcmp(ver, FW_VERSION) != 0) {
				uint32_t d = ota_stagger_ms();

				telem_pub_response(cid, cmd, "ok", NULL,
						   "{\"state\":\"scheduled\"}");
				printk("OTA: fleet update - starting in %u s\n",
				       d / 1000u);
				k_msleep(d);
			}

			int rc = ota_start(url, ver, size);
			char pl[80]; ota_status_json(pl, sizeof(pl));

			/* -EALREADY = already on this version. That is a
			 * SUCCESSFUL no-op for a repeated fleet command, not a
			 * failure - report ok so the back office can mark the
			 * wagon done rather than retrying it forever. */
			if (rc == -EALREADY) {
				telem_pub_response(cid, cmd, "ok", NULL,
						   "{\"state\":\"current\"}");
			} else if (!fleet || rc != 0) {
				telem_pub_response(cid, cmd, rc==0?"ok":"err",
						   rc==0?NULL:"BUSY", pl);
			}
		}
	} else if (!strcmp(cmd, "ota_status")) {
		char pl[96];

		ota_status_json(pl, sizeof(pl));
		telem_pub_response(cid, cmd, "ok", NULL, pl);
	} else if (!strcmp(cmd, "get_status")) {
		telem_pub_heartbeat(motion_is_moving() ? "running" : "stopped");
		telem_pub_response(cid, cmd, "ok", NULL, "{\"applied\":true}");
	} else if (!strcmp(cmd, "learn_fitment")) {
		/*
		 * COMMISSIONING AID: adopt whatever nodes we can actually hear.
		 *
		 * Setting the mask by hand is fine for one wagon and unreasonable
		 * for a fleet where fitments vary - 1 tank here, 2 there, 4
		 * bearings on the next. The fitter would have to work out a
		 * bitmask per wagon and type it in without transposing a digit.
		 * The gateway already knows: it has been listening to those exact
		 * nodes since power-up.
		 *
		 * ONE-SHOT, NEVER CONTINUOUS. It is tempting to keep the roster
		 * in sync with what is heard, and that would be a serious bug: a
		 * node that dies would simply be forgotten instead of reported
		 * missing, which is precisely the fault SENSOR_FAULT exists to
		 * catch. The roster must be a statement of what SHOULD be there,
		 * so it can only ever be set deliberately.
		 *
		 * Hence: run it at commissioning, with every node powered and in
		 * range, and check the reply before trusting it. A node that was
		 * asleep or out of range during the sweep is silently absent from
		 * the result - which is why the response reports the count for
		 * the fitter to compare against the work order.
		 */
		uint32_t mask = 0;
		int heard = 0;

		for (uint8_t id = 0; id < 19; id++) {
			struct sw_node_entry e;

			if (ble_sensors_get(id, &e)) {
				mask |= (uint32_t)1u << id;
				heard++;
			}
		}

		if (mask == 0) {
			telem_pub_response(cid, cmd, "err", "NOT_FOUND",
					   "{\"info\":\"no nodes heard - power "
					   "them and retry\"}");
		} else {
			int rc = cfg_set_fitment(mask);
			char pl[112];

			snprintf(pl, sizeof(pl),
				 "{\"fitment\":%u,\"nodes\":%d,\"image\":%u}",
				 mask, heard, wagon_fitment_image());
			printk("GW: learned fitment 0x%05X from %d node(s)\n",
			       mask, heard);
			telem_pub_response(cid, cmd, rc == 0 ? "ok" : "err",
					   rc == 0 ? NULL : "BAD_PARAM", pl);
		}
	} else if (!strcmp(cmd, "set_interval")) {
		bool ch = false;
		if (json_has(j, "moving_s")) {
			long v = json_num(j, "moving_s");
			if (v >= 30 && v <= 86400) { g_cfg.moving_s = (uint32_t)v; ch = true; }
		}
		if (json_has(j, "idle_s")) {
			long v = json_num(j, "idle_s");
			if (v >= 60 && v <= 604800) { g_cfg.idle_s = (uint32_t)v; ch = true; }
		}
		if (ch) {
			config_save();          /* persist only on real change */
			arm_schedule();         /* apply to the next wake       */
			telem_pub_response(cid, cmd, "ok", NULL, "{\"applied\":true}");
		} else {
			telem_pub_response(cid, cmd, "err", "BAD_PARAM", "{}");
		}
	} else if (!strcmp(cmd, "set_threshold")) {
		char param[24] = "";
		json_str(j, "param", param, sizeof(param));

		/*
		 * WHO is being addressed - the gateway itself, or sub-nodes?
		 *
		 * The presence of a NODE SELECTOR decides. "impact_g" is a valid
		 * parameter on both sides (the gateway has its own BMA400, and so
		 * do the door and tank nodes), so without this test the gateway
		 * branch below would swallow every sub-node impact command and
		 * answer applied:true having changed nothing on any node.
		 */
		bool to_nodes = json_has(j, "node") || json_has(j, "type") ||
				json_has(j, "all");

		/*
		 * FITMENT - retro-fit a sensor without rebuilding for one wagon.
		 *
		 * The mask compiled into the image describes the wagon as it left
		 * the works. Fit a door sensor in service and that mask is wrong:
		 * the node advertises, the gateway is not expecting it, and the
		 * roster still reports a node missing. Rebuilding one image for
		 * one wagon is exactly the per-wagon build this whole change set
		 * exists to remove.
		 *
		 * So the mask is settable here and stored in FRAM. Send 0 to drop
		 * the override and follow the image again - which is also how a
		 * wagon rejoins the fleet default after a sensor is removed.
		 *
		 * Deliberately NOT routed by the node selector: this is gateway
		 * configuration describing which nodes exist, not a value pushed
		 * to a node.
		 */
		if (!to_nodes && !strcmp(param, "fitment") && json_has(j, "value")) {
			uint32_t mask = (uint32_t)json_num(j, "value");
			int rc = cfg_set_fitment(mask);
			char pl[96];

			snprintf(pl, sizeof(pl),
				 "{\"fitment\":%u,\"image\":%u,\"nodes\":%d}",
				 wagon_fitment_id(), wagon_fitment_image(),
				 wagon_node_count());
			telem_pub_response(cid, cmd, rc == 0 ? "ok" : "err",
					   rc == 0 ? NULL : "BAD_PARAM", pl);
			return;
		}

		if (!to_nodes && !strcmp(param, "impact_g") && json_has(j, "value")) {
			double v = json_dbl(j, "value");
			if (v > 0.5 && v <= 16.0) {
				g_cfg.impact_mg = (uint16_t)(v * 1000.0);
				config_save();
				telem_pub_response(cid, cmd, "ok", NULL, "{\"applied\":true}");
			} else {
				telem_pub_response(cid, cmd, "err", "BAD_PARAM", "{}");
			}
		} else if ((json_has(j, "all") || json_has(j, "type")) &&
			   json_has(j, "value")) {
			/*
			 * BULK / PER-TYPE sub-node threshold.
			 *
			 *   "all": true            -> every node heard on this wagon
			 *   "type": <sw_node_type> -> every node of that type
			 *   both                   -> same as "type" (type narrows)
			 *
			 * This is the multi-node counterpart of the single-node
			 * form below: a rake-wide threshold change would otherwise
			 * need one command per node, each waiting for that node's
			 * ~4 s connectable window, with no way to tell how many
			 * were actually reachable.
			 *
			 * Values are in the NODE's raw units, exactly as for the
			 * single-node form - degC x10 for bearing and tank nodes.
			 * A "type" scope is the safe way to use this: sending one
			 * value to ALL types is rarely meaningful, because 600
			 * means 60.0 C to a tank node and a nonsense load percent
			 * to a load node.
			 */
			int16_t v  = (int16_t)json_dbl(j, "value");
			int16_t h  = json_has(j, "hyst") ?
				     (int16_t)json_dbl(j, "hyst") : 0;
			int  wtype = json_has(j, "type") ?
				     (int)json_dbl(j, "type") : -1;
			bool all   = json_bool(j, "all", false);
			uint8_t dn = dn_cmd_for_param(param);
			/* -1 for every param that is not gauge calibration; the
			 * frame's rsvd byte is 0 in that case, as it always was. */
			int  sl    = dn_battcal_sel(param);
			uint8_t bsel = (sl >= 0) ? (uint8_t)sl : 0;

			if (dn == 0) {
				telem_pub_response(cid, cmd, "err", "BAD_PARAM",
						   "{\"info\":\"unknown param\"}");
				return;
			}

			/*
			 * node_id is the ONE parameter a broad scope must never
			 * carry.
			 *
			 * Every other setting is a value each node applies to
			 * itself, so sending it to a whole type is exactly what
			 * scope is for. An id is the opposite: it is what tells
			 * nodes APART. A type-scoped node_id would set all eight
			 * bearings to the same id, and the moment they did, the
			 * gateway could no longer address any of them
			 * individually to undo it - the wagon would need every
			 * bearing reflashed over SWD.
			 *
			 * This branch handles the type/all scopes, so reaching
			 * here with SET_NODEID is always wrong.
			 */
			if (dn == SW_DN_SET_NODEID) {
				telem_pub_response(cid, cmd, "err", "BAD_PARAM",
					"{\"info\":\"node_id needs an explicit "
					"'node' - a type or all scope would give "
					"every node the same id and leave none "
					"of them addressable\"}");
				return;
			}

			int bad = 0;
			int n   = queue_threshold_scope(-1, wtype, all, dn,
							bsel, v, h, &bad);
			char pl[144];

			snprintf(pl, sizeof(pl),
				 "{\"applied\":false,\"state\":\"queued\","
				 "\"nodes\":%d,\"out_of_range\":%d}", n, bad);
			if (n > 0) {
				/*
				 * Partial success stays "ok" - the nodes that can
				 * take the value have it queued. out_of_range is
				 * reported either way, because a mixed-type scope
				 * where the doors silently dropped out is exactly
				 * the case an operator would never otherwise
				 * notice.
				 */
				telem_pub_response(cid, cmd, "ok", NULL, pl);
			} else if (bad > 0) {
				/*
				 * Nothing queued and every candidate rejected the
				 * VALUE. Reporting NOT_FOUND here would send the
				 * operator to check power and antennas for a node
				 * that is present, awake, and simply cannot reach
				 * the number they sent.
				 */
				telem_pub_response(cid, cmd, "err",
						   "INVALID_THRESHOLD", pl);
			} else {
				/* Nothing matched: either no node of that type is
				 * provisioned, or none has been heard yet. */
				telem_pub_response(cid, cmd, "err", "NOT_FOUND",
						   pl);
			}
		} else if (json_has(j, "node") && json_has(j, "value")) {
			/*
			 * Node-held threshold (bearing over-temp, tilt, tank temp).
			 * The target node is only reachable during its ~4 s
			 * connectable window, roughly every 10 minutes - far longer
			 * than the broker will wait for a response. So queue it and
			 * answer "queued", not "applied". node_link delivers it over
			 * BLE when the observer next sees that node's window; the
			 * node's own uplink then reflects the new setting.
			 */
			int nid = (int)json_dbl(j, "node");
			int16_t v = (int16_t)json_dbl(j, "value");
			int16_t h = json_has(j, "hyst") ? (int16_t)json_dbl(j, "hyst") : 0;
			struct sw_node_entry ne;
			uint8_t ntype = ble_sensors_get((uint8_t)nid, &ne) ?
					ne.data.node_type : 0;

			uint8_t dn = dn_cmd_for_param(param);
			int     sl = dn_battcal_sel(param);
			uint8_t bsel = (sl >= 0) ? (uint8_t)sl : 0;

			if (nid < 0 || nid >= SW_MAX_NODES || dn == 0) {
				telem_pub_response(cid, cmd, "err", "BAD_PARAM", "{}");
			} else if (ntype == 0) {
				/* Never heard from it: we cannot seal a frame without
				 * knowing the node type (it is authenticated in the AAD). */
				telem_pub_response(cid, cmd, "err", "NOT_FOUND", "{}");
			} else if (dn == SW_DN_SET_BATTCAL &&
				   !sw_battcal_sane(bsel, v)) {
				/*
				 * Refuse here rather than let the node do it.
				 * The node also checks, but its only way to say
				 * no is a GATT error up to ten minutes from now,
				 * long after this response was published - so
				 * this is the last point where a bad value can
				 * still reach the operator as an MQTT error.
				 */
				telem_pub_response(cid, cmd, "err",
						   "INVALID_THRESHOLD",
						   "{\"info\":\"calibration "
						   "value out of range\"}");
			} else if (dn == SW_DN_SET_BATT && !sw_batt_mv_sane(v, h)) {
				telem_pub_response(cid, cmd, "err",
						   "INVALID_THRESHOLD",
						   "{\"info\":\"need 2000..4000 mV, "
						   "fresh-eol >= 100\"}");
			} else if (node_link_queue_cmd_sel((uint8_t)nid, ntype, dn,
							   bsel, v, h) == 0) {
				telem_pub_response(cid, cmd, "ok", NULL,
						   "{\"applied\":false,\"state\":\"queued\"}");
			} else {
				telem_pub_response(cid, cmd, "err", "BAD_PARAM", "{}");
			}
		} else {
			telem_pub_response(cid, cmd, "err", "BAD_PARAM", "{}");
		}
	} else if (!strcmp(cmd, "node_window")) {
		/*
		 * Put sub-nodes into a temporary FAST config window.
		 *
		 * A node is normally reachable for 4 s in every 600 s, so a queued
		 * threshold waits ~5 minutes on average. That is correct for a
		 * fit-and-forget node but painful while commissioning a rake. This
		 * asks the targeted nodes to open their window every ~5 s for a
		 * bounded number of seconds, then revert BY THEMSELVES.
		 *
		 * The node caps and deadlines it (CFGWIN_FAST_MAX_S), and never
		 * persists it, so nothing here can strand a node at high duty:
		 * worst case it pays the fast rate until its own timer expires.
		 *
		 * Uses the same node selector as set_threshold, so "seconds" can be
		 * aimed at one node, one type, or every node heard.
		 */
		int secs = json_has(j, "seconds") ? (int)json_dbl(j, "seconds") : 0;
		int  wtype = json_has(j, "type") ? (int)json_dbl(j, "type") : -1;
		bool all   = json_bool(j, "all", false);
		int  nid   = json_has(j, "node") ? (int)json_dbl(j, "node") : -1;

		if (secs < 0 || secs > CFGWIN_FAST_MAX_REQ_S ||
		    (nid < 0 && wtype < 0 && !all)) {
			telem_pub_response(cid, cmd, "err", "BAD_PARAM",
					   "{\"info\":\"seconds 0-3600 and a node/type/all selector\"}");
		} else {
			int n = queue_threshold_scope(nid, wtype, all,
						      SW_DN_FASTWIN, 0,
						      (int16_t)secs, 0, NULL);
			char pl[96];

			snprintf(pl, sizeof(pl),
				 "{\"applied\":false,\"state\":\"queued\","
				 "\"nodes\":%d,\"seconds\":%d}", n, secs);
			telem_pub_response(cid, cmd, n > 0 ? "ok" : "err",
					   n > 0 ? NULL : "NOT_FOUND", pl);
		}
	} else if (!strcmp(cmd, "set_gnss")) {
		if (json_has(j, "enable")) {
			g_cfg.gnss_enable = json_bool(j, "enable", g_cfg.gnss_enable) ? 1 : 0;
		}
		if (json_has(j, "fix_timeout_s")) {
			long v = json_num(j, "fix_timeout_s");
			if (v >= 10 && v <= 300) g_cfg.gnss_timeout_s = (uint16_t)v;
		}
		if (json_has(j, "constellation")) {
			char c[16] = ""; json_str(j, "constellation", c, sizeof(c));
			g_cfg.gnss_constel = !strcmp(c, "navic") ? 1 :
					     !strcmp(c, "gps")   ? 2 :
					     !strcmp(c, "navic_gps") ? 3 : 0;
			/* GNSS rail is powered here (mid-report, after acquire_fix), so
			 * push the selection to the LC29H now; PAIR513 persists it. */
			gnss_set_constellation(g_cfg.gnss_constel);
		}
		config_save();
		telem_pub_response(cid, cmd, "ok", NULL, "{\"applied\":true}");
	} else if (!strcmp(cmd, "time_sync")) {
		long e = json_num(j, "epoch");
		if (e > 0) {
			telem_set_time((uint32_t)e);   /* set the software clock */
			telem_pub_response(cid, cmd, "ok", NULL, "{\"applied\":true}");
		} else {
			telem_pub_response(cid, cmd, "err", "BAD_PARAM", "{}");
		}
	} else if (!strcmp(cmd, "set_wagon")) {
		/*
		 * RDSO S7.18/S7.19 provisioning: set the wagon number in FRAM.
		 *
		 * REBOOTS, and must. The wagon number is not a value read where it
		 * is used - it seeds the MQTT client id, the topic prefix, the BLE
		 * isolation group (CRC16) and the per-wagon AES-CCM key (HKDF),
		 * all computed once at startup. Rebuilding those in place would
		 * mean tearing down the broker session and the BLE filter
		 * mid-cycle; a restart is simpler and provably consistent.
		 *
		 * ONE-WAY OVER THE AIR. After the reboot this gateway answers on
		 * the NEW wagon's topic and is deaf to the old one - so a command
		 * sent with the wrong number does not fail, it moves the gateway
		 * somewhere the operator may not be listening. Recovery is to
		 * publish to the new topic, which requires knowing what was sent.
		 * The response below carries the new number for exactly that
		 * reason: it is the last thing the old topic ever hears.
		 */
		char w[16] = "";

		json_str(j, "wagon", w, sizeof(w));

		int rc = cfg_set_wagon(w);

		if (rc != 0) {
			telem_pub_response(cid, cmd, "err", "BAD_PARAM",
					   "{\"info\":\"wagon must be 1..15 "
					   "characters\"}");
		} else {
			char pl[96];

			snprintf(pl, sizeof(pl),
				 "{\"wagon\":\"%s\",\"grp\":%u,"
				 "\"rebooting\":true}",
				 cfg_wagon(), sw_wgn_group(cfg_wagon()));
			telem_pub_response(cid, cmd, "ok", NULL, pl);
			printk("GW: wagon number set to %s - rebooting\n",
			       cfg_wagon());
			k_msleep(500);
			sys_reboot(SYS_REBOOT_COLD);
		}
	} else if (!strcmp(cmd, "set_batt")) {
		/*
		 * Re-map the LTO pack's state of charge: full_mv -> 100 %,
		 * empty_mv -> 0 %.
		 *
		 * Settable over the air because the right values are a property of
		 * the PACK, not of the firmware. They depend on what the BQ25798
		 * actually terminates at and where the power mux hands over to the
		 * Li-SOCl2 backup, neither of which is known until a real pack has
		 * been charged and run down on the bench. The compile-time seeds
		 * are an estimate from the cell chemistry; this command is how a
		 * measurement replaces them without a firmware build.
		 *
		 * No reboot: nothing is derived from these two values. The next
		 * charger read uses the new map, so the effect is visible on the
		 * following heartbeat.
		 *
		 * This does NOT reach sub-nodes. Their cell is a different
		 * chemistry with a flat discharge curve and no voltage->percent
		 * map at all - use set_threshold param:"batt_mv" for those.
		 */
		if (!json_has(j, "full_mv") || !json_has(j, "empty_mv")) {
			telem_pub_response(cid, cmd, "err", "BAD_PARAM",
					   "{\"info\":\"need full_mv and "
					   "empty_mv\"}");
			return;
		}

		uint16_t full  = (uint16_t)json_num(j, "full_mv");
		uint16_t empty = (uint16_t)json_num(j, "empty_mv");
		int rc = cfg_set_batt(full, empty);
		char pl[192];

		if (rc != 0) {
			/*
			 * INVALID_THRESHOLD, not BAD_PARAM: the fields were
			 * present and well-formed, the NUMBERS are unusable. The
			 * accepted range travels in the response because the
			 * sender cannot otherwise tell which of the two ends was
			 * refused, or why.
			 */
			snprintf(pl, sizeof(pl),
				 "{\"info\":\"full_mv and empty_mv must be "
				 "%u..%u mV and at least %u mV apart\","
				 "\"full_mv\":%u,\"empty_mv\":%u}",
				 GW_BATT_MV_MIN, GW_BATT_MV_MAX,
				 GW_BATT_MV_SPAN,
				 cfg_batt_full_mv(), cfg_batt_empty_mv());
			telem_pub_response(cid, cmd, "err",
					   "INVALID_THRESHOLD", pl);
		} else {
			struct charger_status cs;
			bool have = (charger_read(&cs) == 0);

			snprintf(pl, sizeof(pl),
				 "{\"applied\":true,\"full_mv\":%u,"
				 "\"empty_mv\":%u,\"vbat_mv\":%u,"
				 "\"soc\":%u}",
				 cfg_batt_full_mv(), cfg_batt_empty_mv(),
				 have ? cs.vbat_mv : 0, have ? cs.soc : 0);
			telem_pub_response(cid, cmd, "ok", NULL, pl);
		}
	} else if (!strcmp(cmd, "reboot")) {
		telem_pub_response(cid, cmd, "ok", NULL, "{}");
		k_msleep(500);
		sys_reboot(SYS_REBOOT_COLD);
	} else if (!strcmp(cmd, "get_config")) {
		/* Protocol s6.2: read back current config. args{section} is accepted but
		 * ignored - the whole block fits in one response, so splitting it
		 * would only add a round trip on a Class-A link. */
		char pl[288];   /* grew with batt_full_mv / batt_empty_mv */

		snprintf(pl, sizeof(pl),
			 "{\"run_s\":%u,\"stop_s\":%u,\"impact_mg\":%u,"
			 "\"gnss_enable\":%u,\"gnss_constel\":%u,\"gnss_timeout_s\":%u,"
			 "\"nodes\":%d,\"fitment\":%u,\"wagon\":\"%s\","
			 "\"batt_full_mv\":%u,\"batt_empty_mv\":%u,"
			 "\"fw\":\"%s\"}",
			 g_cfg.moving_s, g_cfg.idle_s, g_cfg.impact_mg,
			 g_cfg.gnss_enable, g_cfg.gnss_constel, g_cfg.gnss_timeout_s,
			 wagon_node_count(), wagon_fitment_id(), cfg_wagon(),
			 cfg_batt_full_mv(), cfg_batt_empty_mv(),
			 FW_VERSION);
		telem_pub_response(cid, cmd, "ok", NULL, pl);
	} else if (!strcmp(cmd, "get_history")) {
		/*
		 * Protocol s6.2/s8.4: PULL replay of buffered records. The normal path is
		 * push (telem_flush_backlog on reconnect); this exists for the cloud to
		 * request a backlog it believes it missed.
		 *
		 * Only the COUNT and range are returned here, not the records
		 * themselves: each buffered record is a full envelope up to ~640 B and
		 * the response would blow past the modem's publish buffer. The cloud
		 * gets the records via the normal flush, which republishes them with
		 * their ORIGINAL seq and ts so (gw,seq) dedup still works (s1.3).
		 */
		char pl[128];
		int n = storage_count();

		snprintf(pl, sizeof(pl),
			 "{\"from_seq\":%ld,\"count\":%d,\"mode\":\"push\","
			 "\"info\":\"records replay on reconnect with original seq/ts\"}",
			 json_num(j, "from_seq"), n);
		telem_pub_response(cid, cmd, "ok", NULL, pl);
		if (n > 0) {
			telem_flush_backlog();
		}
	} else if (!strcmp(cmd, "pull_data")) {
		/* Protocol s6.2/s7.20: on-demand raw WSN snapshot. Answered by forcing a
		 * heartbeat, which already carries the full sens[] array - duplicating
		 * that formatting here would risk the two diverging. */
		char pl[96];

		snprintf(pl, sizeof(pl), "{\"nodes\":%d,\"via\":\"hb\"}",
			 ble_sensors_snapshot(NULL, 0) >= 0 ? wagon_node_count() : 0);
		telem_pub_response(cid, cmd, "ok", NULL, pl);
		telem_pub_heartbeat(motion_is_moving() ? "running" : "stopped");
	} else if (!strcmp(cmd, "provision_node")) {
		/*
		 * Protocol s6.2/s7.18-7.19: map a sensor node to this wagon.
		 *
		 * NOT SUPPORTED at runtime by design. The roster lives in nodes.c as a
		 * compile-time table (WAGON_NODES) because node_id must agree with the
		 * NODE_ID compiled into each sub-node, and the canonical id<->position
		 * map is fleet-wide (see DOCS/NODE_MAP.md). Accepting a runtime
		 * remap would let the gateway and the node disagree about which wheel a
		 * reading belongs to - silently, and fleet-wide.
		 */
		telem_pub_response(cid, cmd, "err", "UNSUPPORTED",
				   "{\"info\":\"roster is compile-time (nodes.c)\"}");
	} else {
		telem_pub_response(cid, cmd, "err", "UNSUPPORTED", "{}");
	}
}

/* Class-A: after every uplink, drain queued commands from dn/cmd. */
static void drain_commands(void)
{
	char cmdbuf[512];
	while (ec200_mqtt_poll_cmd(cmdbuf, sizeof(cmdbuf), 1500) == 1) {
		handle_command(cmdbuf);
		ota_tick();
	}
	ota_tick();
}

/* ---- power-sequenced fix + link (Class-A wake cycle) ---- */

/* Power the GNSS rail on and block (no poll) until a fresh fix or timeout. */
static void acquire_fix(int timeout_ms)
{
	if (!g_cfg.gnss_enable) {          /* GNSS disabled by config: last-known */
		return;
	}
	gnss_power_on();
	if (gnss_wait_fix(timeout_ms) != 0) {
		printk("GW: fix timeout - using last-known position\n");
	}
}
static void release_fix(void) { gnss_power_off(); }

/*
 * Modem session policy.
 *
 * PRODUCTION cycles the modem for every report: it is by far the largest
 * consumer, and at a 10 minute cadence (s7.4) the attach cost is amortised
 * over a long idle, so powering it down is unambiguously right.
 *
 * ON THE BENCH that same behaviour makes a short cadence unusable. A cold
 * attach - PWRKEY, boot, network registration, PDP activation, MQTT CONNECT -
 * commonly takes 10-30 s, so at a 30 s cadence the gateway spends nearly all
 * its time attaching and may still be mid-attach when the next report falls
 * due. You end up debugging the attach sequence instead of the thing you were
 * trying to watch.
 *
 * So under DEBUG_TRACE the session is HELD UP between reports. The first
 * link_up() does the full sequence; later ones verify the session is still
 * alive and return immediately, and link_down() becomes a no-op. If the
 * session has dropped - lost registration, broker timeout - the check fails
 * and the full sequence runs again, so a real outage still recovers.
 */


static int link_bringup(void)
{
	/*
	 * Rails first, then ASK before pressing the button.
	 *
	 * PWRKEY is a toggle: pulsing a module that is already running powers it
	 * down. The modem survives a reset that does not pass through
	 * gnss_power_off() - a fresh flash, a watchdog bite, a brownout - so an
	 * unconditional pulse here switched off a working modem and then spent
	 * every retry asking a dead module to answer.
	 */
	modem_power_on();                /* rails, translator, reset deasserted */

	/*
	 * NO PWRKEY HERE - ec200_bringup() owns it.
	 *
	 * This used to do its own polled check and pulse PWRKEY when silent, and
	 * ec200_bringup() (called from ec200_modem_up below) does exactly the
	 * same thing. Both ran, so every cycle pulsed TWICE - and PWRKEY is a
	 * toggle, so the first pulse started the module and the second switched
	 * it straight back off. The console showed it plainly once both were
	 * logging: two "pulsing PWRKEY" lines per cycle, and the NETLIGHT coming
	 * on at the second one.
	 *
	 * One owner for a toggle. The bring-up checks first and pulses only if
	 * the module is genuinely silent.
	 *
	 * And it is called ONCE, here, OUTSIDE the retry loop below. Calling it
	 * from inside ec200_modem_up() put it inside those retries, so a cycle
	 * pulsed up to three times - on, off, on - and the module ended up
	 * switched off with the log showing two "pulsing PWRKEY" lines and the
	 * NETLIGHT lighting at the second.
	 */
	/*
	 * modem_simple decides. It owns PWRKEY, the AT handshake and the broker
	 * session, and it is the only path that has ever attached from this
	 * board.
	 *
	 * ec200_bringup() used to run first and pulse PWRKEY itself. It always
	 * failed - its reads never see a byte here - and because PWRKEY is a
	 * TOGGLE, modem_simple then pulsed a second time: two 20 s boot settles
	 * per cycle, and a live risk of switching a running module back off.
	 *
	 * Worse, its failure became the gateway's `online` flag. The backlog
	 * flush and the command drain are both gated on that flag, so neither
	 * ever ran: alarms reached the broker (telem_send publishes directly)
	 * while heartbeats accumulated unsent in the ring.
	 */
	if (modem_simple_up() != 0) {
		return -1;
	}

	/*
	 * ec200 still gets a turn, but ONLY now that the module is awake and
	 * attached, and its result no longer decides anything. It owns the
	 * QMTSUB subscription that downlink commands arrive on, and the HTTP
	 * download used for OTA - neither of which modem_simple implements.
	 *
	 * Best-effort on purpose: a gateway that can publish telemetry but not
	 * receive commands is degraded, not offline, and reporting it as offline
	 * would stop the very uplink that still works.
	 */
	/*
	 * Retained "online" as soon as the session is real.
	 *
	 * This used to be published by ec200.c, whose connect never succeeded on
	 * this board - so the retained status said "offline" permanently while the
	 * gateway was publishing telemetry perfectly well. Sending it from the
	 * path that owns the session is the only way it can be true.
	 */
	{
		char t[96];

		char sp[64];

		snprintf(t, sizeof(t), "%s/%s/%s/status",
			 TOPIC_ROOT, cfg_wagon(), cfg_gw_id());
		/* Rev.1 s7.3 birth: the OBJECT {"st":"online","ts":...}, retained.
		 * This was a bare string, which no subscriber built to the spec
		 * can parse and which carried no timestamp. */
		snprintf(sp, sizeof(sp), "{\"st\":\"online\",\"ts\":%u}",
			 (unsigned)telem_epoch());
		(void)modem_simple_publish_retain(t, sp);
	}

	if (ec200_mqtt_up() != 0) {
		printk("LINK: uplink OK, but ec200 subscribe failed - "
		       "downlink commands may not arrive this cycle\n");
	}
	return 0;
}

/* Power the modem on and connect (one session per report cycle). */
static int link_up(void)
{
	return link_bringup();
}

/*
 * Graceful disconnect + power the modem down (biggest consumer on the board).
 *
 * This ALWAYS runs now. There used to be a MODEM_HOLD_SESSION escape that kept
 * the module attached between cycles; it was a crutch for a transport broken by
 * reversed uart22 pins in app.overlay, and with that fixed it only burns the
 * s5.1.20 energy budget - a held LTE session costs tens of milliamps around the
 * clock, against a duty cycle that assumes the modem is off ~99 % of the time.
 *
 * Order matters: QPOWD first so the module tears the PDP context down and tells
 * the network, THEN the rail. Cutting VBAT on an attached modem leaves a stale
 * context at the carrier and makes the next attach slower.
 */
static void link_down(void)
{
	/*
	 * Retained "offline" before tearing the session down.
	 *
	 * `status` tracks the LINK: "online" while the broker session is up,
	 * "offline" from here, and "offline" via the Last Will if the session
	 * ever dies without a DISCONNECT. All three paths are covered.
	 *
	 * Consequence to design the back office around: the gateway is
	 * deliberately disconnected most of the time - about 99 % at the RDSO
	 * §7.4 ten-minute cadence - so this topic reads "offline" almost whenever
	 * anyone looks. That is correct, not a fault. A cycling gateway and a
	 * dead one both show "offline", so liveness must come from staleness on
	 * the heartbeat `ts`, not from this field.
	 *
	 * Published through modem_simple because it owns the session and is the
	 * only path whose publishes reach the broker on this board. Must run
	 * BEFORE ec200_disconnect(), which closes that session.
	 */
	if (modem_simple_is_up()) {
		char t[96];

		char sp[64];

		snprintf(t, sizeof(t), "%s/%s/%s/status",
			 TOPIC_ROOT, cfg_wagon(), cfg_gw_id());
		snprintf(sp, sizeof(sp), "{\"st\":\"offline\",\"ts\":%u}",
			 (unsigned)telem_epoch());
		(void)modem_simple_publish_retain(t, sp);
	}

	ec200_disconnect();
	modem_power_off();

	/* The rail is down, so the broker session is gone with it. Say so, or the
	 * next cycle opens believing it still has one and burns two AT timeouts
	 * rediscovering that the modem has no power. */
	modem_simple_session_closed();
}

int main(void)
{
	/*
	 * IDENTITY FIRST - before anything derives from the wagon number.
	 *
	 * The BLE isolation group and the per-wagon AES-CCM key are computed
	 * once, a few lines below and at sw_secure_init(). An earlier revision
	 * kept the wagon number in app_cfg, which config_load() does not read
	 * until much later in this function - so both were derived from the
	 * compile-time seed while the provisioned value sat unread. Nothing
	 * failed visibly: the gateway simply published to the wrong wagon's
	 * topic and could not decrypt a single one of its own nodes.
	 *
	 * Hence on-chip NVS and hence here, first. It also removes the FRAM
	 * from the identity path entirely, so a board whose external memory is
	 * missing or whose VCC_MEM has failed still knows which wagon it is.
	 */
	cfg_identity_load();

	uint16_t wgn_group = sw_wgn_group(cfg_wagon());   /* derived from wagon no */
	/*
	 * WHY DID WE RESTART?
	 *
	 * On nRF a genuine power-on reset leaves RESETREAS with NO bits set;
	 * every other cause sets its own. So "none" means the rail actually
	 * dropped - a brownout or a real power cycle - which on a supply that
	 * cannot hold up under the modem's attach current is by far the most
	 * likely explanation for an unexplained reboot.
	 *
	 * Printed before the banner so it heads every boot log, and cleared
	 * immediately so the NEXT boot reports its own cause rather than
	 * inheriting this one.
	 */
	{
		uint32_t cause = 0;
		int rc = hwinfo_get_reset_cause(&cause);

		(void)hwinfo_clear_reset_cause();

		if (rc != 0) {
			printk("RESET: cause unavailable (%d)\n", rc);
		} else if (cause == 0) {
			printk("RESET: POWER (brownout or rail cycled) "
			       "- check the supply before anything else\n");
		} else {
			printk("RESET: 0x%08X%s%s%s%s\n", cause,
			       (cause & RESET_WATCHDOG) ? " WATCHDOG" : "",
			       (cause & RESET_SOFTWARE) ? " SOFTWARE" : "",
			       (cause & RESET_DEBUG)    ? " DEBUG"    : "",
			       (cause & RESET_PIN)      ? " PIN"      : "");
		}
	}

	printk("\n=== Smart Wagon Gateway (wgn %s, grp 0x%04X) ===\n",
	       cfg_wagon(), wgn_group);

	/*
	 * Print the fitment id at boot.
	 *
	 * It is the address a group OTA is aimed at, so whoever prepares a
	 * release needs it - and reading it back over MQTT means having a
	 * working uplink first, which is exactly what you do not have while
	 * commissioning a wagon on the bench. One line here costs nothing and
	 * removes the chicken-and-egg.
	 *
	 * "seeded" means the wagon number came from the compile-time default
	 * because FRAM held nothing valid - worth seeing, since an unprovisioned
	 * unit and a correctly provisioned one otherwise look identical.
	 */
	printk("=== fitment 0x%05X (%u), %d node(s), fw %s, wagon %s ===\n",
	       wagon_fitment_id(), wagon_fitment_id(), wagon_node_count(),
	       FW_VERSION,
	       cfg_wagon());
	nodes_check_convention();

	power_init();            /* rails safe; BMA400 (impact wake) powered */
	gw_wdt_init();          /* internal WDT31: no external watchdog on this board */

	k_event_init(&ev);
	telem_init();
	motion_init(on_motion_confirmed);        /* debounced motion gatekeeper */
	gnss_init();
	ec200_init();
	if (sw_secure_init(cfg_wagon()) != 0) {  /* per-wagon BLE decryption key */
		printk("GW: crypto init failed - sensor adverts cannot be read\n");
	}
	ble_sensors_set_group(wgn_group);        /* <-- multi-wagon isolation */
	ble_sensors_init();                      /* BLE observer stays alive
						  * (System ON idle) to hear alarms */
	ble_sensors_register_alarm_cb(on_node_alarm);
	impact_init();
	node_link_init();       /* BLE downlink to sub-nodes (queued delivery) */
	gwalarm_init();        /* gateway-level alarm/event edge detection */

	bma400_init();           /* ±16g wide range, impact INT -> P1.16 */
	charger_init();          /* BQ25798 on i2c21 -> real battery telemetry */
	/*
	 * Report a genuine failure. This no longer fires just because the FRAM
	 * is unfitted - the ring reconstructs itself from the record sequence
	 * numbers in that case and works normally. It fires only when the NOR
	 * archive partition itself could not be opened, which really does mean
	 * anything that fails to publish is gone.
	 */
	if (storage_init() != 0) {
		printk("MAIN: store-and-forward UNAVAILABLE - offline records "
		       "will be LOST\n");
	}
	thermal_init();          /* s5.1.10 thermal transmit halt          */
	config_load();           /* server-settable cfg from FRAM (or seed) */

	/* Seed the debounce from one reading so the first cadence is sensible.
	 * If the wagon is already rolling at power-up this starts a pending
	 * transition; the poll timer times it to a confirmed START. */
	motion_sample(bma400_is_moving());
	if (motion_active()) { motion_poll_start(); }

	/*
	 * WATCHDOG: declare the whole first cycle as a long, EXPECTED blocking
	 * window before entering it.
	 *
	 * feed_fn() only feeds while awake if s_progress moved since its last
	 * check, and s_progress is bumped by gw_wdt_alive() - which is not
	 * reached until the main loop below. acquire_fix(120000) alone blocks for
	 * exactly GW_WDT_TIMEOUT_MS, so the watchdog sees no progress and resets
	 * the SoC at 120 s, BEFORE "GW: fix timeout" can even print. The symptom
	 * is a boot loop that stops at "config: seeded defaults" every time -
	 * which happens on any bench with no LC29H attached, and in the field any
	 * time a fix or a modem attach is slow.
	 *
	 * In idle mode feed_fn() judges elapsed-vs-budget instead of progress,
	 * which is exactly what a known-long operation needs.
	 */
	gw_wdt_idle_begin(GW_REPORT_BUDGET_MS);

	/* First wake cycle: fix -> link up -> BOOT event + heartbeat -> drain. */
	/*
	 * First fix gets longer than the steady-state timeout: a cold receiver
	 * has no almanac and needs to download one. On the bench it will not fix
	 * indoors at all, so waiting the full two minutes before the modem is
	 * even powered just delays the part being debugged.
	 */
	acquire_fix(DEBUG_TRACE ? 40000 : 120000);
	bool online = (link_up() == 0);
	telem_pub_event("BOOT", NULL, NULL, NULL);
	ble_sensors_refresh(BLE_REFRESH_MS);   /* short active scan boost */
	telem_pub_heartbeat(motion_is_moving() ? "running" : "stopped");
	telem_check_node_health();             /* SENSOR_FAULT for down nodes */
	if (online) {
		telem_flush_backlog();
		drain_commands();
		/* Healthy online cycle: confirm the running image so MCUboot makes a
		 * freshly-swapped OTA image permanent. No-op on a normal boot. */
		if (!boot_is_img_confirmed()) {
			boot_write_img_confirmed();
		}
	}
	link_down();
	release_fix();
	gw_wdt_idle_end();       /* long boot window over */
	gw_wdt_alive();          /* first real progress mark */
	arm_schedule();

	while (1) {
		/* Sleep in System ON idle until a wake source fires. The BLE
		 * scanner and BMA400 INT stay armed the whole time. */
		uint32_t bits = k_event_wait(&ev,
			EV_SCHED|EV_IMPACT|EV_NALARM|EV_MOTION|EV_MPOLL,
			false, K_FOREVER);
		gw_wdt_idle_end();
		k_event_clear(&ev, bits);
		gw_wdt_alive();

		/*
		 * Re-evaluate the thermal halt once per wake, BEFORE any branch that
		 * might transmit (s5.1.10). Sampling here rather than inside
		 * telem_send() means one reading governs the whole pass, so a
		 * heartbeat cannot be half-sent across a state change.
		 */
		thermal_poll();

		/* === LIGHTWEIGHT PATH (no radios): time the motion debounce ===
		 * These wakes only touch the always-on BMA400 over I2C. They do
		 * NOT power the GNSS or modem. A confirmed change posts EV_MOTION,
		 * which then takes the report path on the next loop pass. */
		if (bits & EV_MPOLL) {
			motion_sample(bma400_is_moving());
			if (!motion_active()) {
				k_timer_stop(&motion_poll_timer);  /* firmly stopped */
			}
			bits &= ~EV_MPOLL;
		}
		if (bits & EV_IMPACT) {
			/* BMA400 INT: latch magnitude, feed the debounce, and keep
			 * the poll running so a real start gets timed. Only a true
			 * shock (>= IMPACT_G_THRESH) needs to be reported now; plain
			 * activity is handled by the debounce without waking radios. */
			impact_g = bma400_magnitude_g();
			motion_sample(true);
			motion_poll_start();
			if (impact_g < (g_cfg.impact_mg / 1000.0)) {
				bits &= ~EV_IMPACT;   /* sub-threshold: no report */
			}
		}
		if (!bits) {
			continue;   /* nothing needs the radios - back to sleep */
		}

		/* === REPORT PATH: power up only what this report needs ===
		 * Same watchdog reasoning as the boot cycle: gw_wdt_alive() is
		 * called once per loop pass, but everything below (GNSS fix, modem
		 * attach, MQTT publishes) blocks far longer than the 120 s progress
		 * window, so the report would trip the watchdog on real hardware
		 * whenever the network was slow. Re-enter the idle budget. */
		gw_wdt_idle_begin(GW_REPORT_BUDGET_MS);
		acquire_fix((int)g_cfg.gnss_timeout_s * 1000);
		online = (link_up() == 0);

		if (bits & EV_MOTION) {
			bool mv = motion_is_moving();   /* debounced, confirmed */
			char trip[24];
			snprintf(trip, sizeof(trip), "trip-%u", ++trip_ctr);
			telem_pub_event(mv ? "TRAIN_START" : "TRAIN_STOP",
					mv ? "stopped" : "running",
					mv ? "running" : "stopped", trip);
			doors_reeval();   /* stop -> re-arm door-in-transit latches */
		}
		if (bits & EV_IMPACT) {
			(void)gwalarm_impact(impact_g, motion_is_moving());
		}
		if (bits & EV_NALARM) {
			struct sw_adv na;
			while (alarm_pop(&na)) {   /* drain every queued node alarm */
				telem_pub_node_alarm(&na, motion_is_moving());
			}
			/*
			 * Run the gateway-decided detectors NOW, against the fix
			 * this cycle just took. A bearing vibration escalation
			 * wakes us without queuing anything, so FLAT_WHEEL would
			 * otherwise wait for the next EV_SCHED - up to 10 minutes,
			 * defeating the point of the node going to 1 s.
			 *
			 * Safe to call here as well as in the EV_SCHED branch:
			 * every detector is edge-triggered against its previous
			 * state, so an unchanged condition produces nothing.
			 */
			gwalarm_eval(motion_is_moving());
		}
		if (bits & EV_SCHED) {
			ble_sensors_refresh(BLE_REFRESH_MS);   /* refresh node data */
			doors_reeval();   /* re-arm latches for doors now seen closed */
			telem_pub_heartbeat(motion_is_moving() ? "running":"stopped");
			telem_check_node_health();             /* down -> SENSOR_FAULT */
#if DEBUG_TRACE
			{
				/*
				 * Bench status. Two questions a bring-up keeps
				 * asking - "is the gateway itself alive?" and
				 * "which nodes can it hear?" - answered on the
				 * console once per report so neither needs a
				 * broker subscription to check.
				 */
				struct gnss_fix gf; gnss_get(&gf);
				struct charger_status cs; charger_read(&cs);
				int16_t dt = thermal_die_temp_x10();

				printk("\n"
				       "==== GW %s  up %llds  %s  cadence %us ====" "\n",
				       cfg_wagon(), k_uptime_get() / 1000,
				       online ? "ONLINE" : "offline",
				       motion_is_moving() ? g_cfg.moving_s
							  : g_cfg.idle_s);
				printk("  gnss %s%s  batt %u%% (%u mV)  "
				       "die %d.%dC  queued %d" "\n",
				       gf.valid ? "fix " : "no-fix",
				       gf.valid ? gf.utc_iso : "",
				       cs.valid ? cs.soc : 0,
				       cs.valid ? cs.vbat_mv : 0,
				       dt == THERMAL_NA ? 0 : dt / 10,
				       dt == THERMAL_NA ? 0 : (dt < 0 ? -dt : dt) % 10,
				       storage_count());
				ble_sensors_dump();
			}
#endif
		gwalarm_eval(motion_is_moving());   /* Rev.1 s3.2 / s4.2 detectors */
			/* Re-check motion now so the NEXT cadence is correct, and keep
			 * polling if a transition is in progress. */
			motion_sample(bma400_is_moving());
			if (motion_active()) { motion_poll_start(); }
		}

		/* replay any buffered records, then drain queued commands */
		if (online) { telem_flush_backlog(); drain_commands(); }

		/* --- power everything back down before sleeping --- */
		link_down();
		release_fix();
		gw_wdt_idle_end();   /* report window over */
		gw_wdt_alive();      /* the cycle completed: real progress */
		arm_schedule();
	}
	return 0;
}
