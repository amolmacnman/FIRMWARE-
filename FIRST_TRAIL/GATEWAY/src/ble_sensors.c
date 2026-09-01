/*
 * Gateway BLE observer: passively scans for Smart Wagon sensor-node
 * advertisements, caches the latest reading per node_id, and raises a
 * callback when a node broadcasts an ALARM. Low duty cycle for battery life.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <string.h>

#include "ble_sensors.h"
#include "sw_secure.h"
#include "app_config.h"
#include "nodes.h"

/* BLE scan interval/window are in 0.625 ms units. */
#define BLE_SCAN_UNITS(ms)  ((uint16_t)((ms) * 8 / 5))

static struct sw_node_entry cache[SW_MAX_NODES];
static struct k_mutex       cache_lock;
static void (*alarm_cb)(const struct sw_adv *a, int8_t rssi);
static uint16_t my_group;      /* this wagon's group; 0 = accept all */

/* Anti-replay: last accepted nonce counter per node (must strictly increase). */
static uint32_t last_ctr[SW_MAX_NODES];
static bool     have_ctr[SW_MAX_NODES];

void ble_sensors_set_group(uint16_t wgn_group) { my_group = wgn_group; }

struct parse_ctx { int8_t rssi; const bt_addr_le_t *addr; };

/* Fired when a node opens its CONNECTABLE config/DFU window. The node beacons
 * 4 bytes of manufacturer data (company id + node id + type) - far shorter than
 * sw_adv_enc, so the uplink branch below skips it and the two never collide. */
static void (*window_cb)(uint8_t node_id, uint8_t node_type,
			 const bt_addr_le_t *addr);

void ble_sensors_register_window_cb(void (*cb)(uint8_t, uint8_t,
					       const bt_addr_le_t *))
{
	window_cb = cb;
}

/* Called for each AD structure in an advertisement. */
static bool ad_parse(struct bt_data *data, void *user)
{
	struct parse_ctx *ctx = user;

	/* Sub-node connectable window beacon: company id + node id + node type. */
	if (data->type == BT_DATA_MANUFACTURER_DATA && data->data_len == 4) {
		uint16_t cid = (uint16_t)data->data[0] | ((uint16_t)data->data[1] << 8);

		if (cid == SW_COMPANY_ID && window_cb) {
			window_cb(data->data[2], data->data[3], ctx->addr);
		}
		return false;
	}

	if (data->type == BT_DATA_MANUFACTURER_DATA &&
	    data->data_len >= sizeof(struct sw_adv_enc)) {
		struct sw_adv_enc enc;
		memcpy(&enc, data->data, sizeof(enc));

		/* Cheap cleartext checks first (no crypto for foreign packets). */
		if (enc.company_id != SW_COMPANY_ID ||
		    enc.proto_ver  != SW_PROTO_VER) {
			return true;    /* not ours; keep scanning AD fields */
		}
		/* MULTI-WAGON FILTER: reject other wagons' groups outright. */
		if (my_group != 0 && enc.wgn_group != my_group) {
			return false;
		}
		if (enc.node_id >= SW_MAX_NODES) {
			return false;
		}

		/* Verify + decrypt. A bad tag (tamper / wrong key) -> drop. */
		struct sw_adv a;
		if (sw_secure_open(&enc, &a) != 0) {
			return false;
		}

		k_mutex_lock(&cache_lock, K_FOREVER);
		/* Anti-replay: counter must strictly increase per node. */
		if (have_ctr[enc.node_id] && enc.ctr <= last_ctr[enc.node_id]) {
			k_mutex_unlock(&cache_lock);
			return false;   /* replayed / stale advert */
		}
		last_ctr[enc.node_id] = enc.ctr;
		have_ctr[enc.node_id] = true;

		bool first = !cache[enc.node_id].seen;

		cache[enc.node_id].seen  = true;
		cache[enc.node_id].data  = a;
		cache[enc.node_id].rssi  = ctx->rssi;
		cache[enc.node_id].ts_ms = k_uptime_get();
		k_mutex_unlock(&cache_lock);

#if DEBUG_TRACE
		/*
		 * Bench visibility. Nothing used to print on a SUCCESSFUL receive,
		 * so a working link and a dead one looked identical on the console
		 * and the only evidence was a heartbeat several minutes later.
		 *
		 * Reaching this point means far more than "a packet arrived": the
		 * company id, protocol version and wagon group all matched, the
		 * AES-CCM tag verified against the per-wagon key, and the replay
		 * counter advanced. So this line is proof the whole security chain
		 * lines up between this gateway and that node - which is exactly
		 * what a first bring-up needs to confirm.
		 */
		printk("RX %s node %u (%s)  val %d  val2 %d  batt %u%%  "
		       "flags 0x%02X  rssi %d\n",
		       first ? "FIRST" : "     ", enc.node_id,
		       sw_typ_str(a.node_type), a.value, a.value2, a.batt,
		       a.flags, ctx->rssi);
#endif

		/* Forward to the adjudicator (main.c) when the subnode already flags
		 * ALARM (impact / tamper / over-temp), OR when a DOOR node reports it
		 * is OPEN - the gateway itself decides whether open-while-moving is an
		 * unauthorised-door alarm, using its authoritative motion + geofence. */
		bool actionable = (a.flags & SW_FLAG_ALARM) ||
				  (a.node_type == SW_TYPE_DOOR &&
				   (a.flags & SW_FLAG_DOOR_OPEN));
		if (actionable && alarm_cb) {
			alarm_cb(&a, ctx->rssi);
		}
		return false;   /* found ours; stop parsing this advert */
	}
	return true;   /* keep scanning AD fields */
}

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
		    struct net_buf_simple *buf)
{
	ARG_UNUSED(adv_type);
	struct parse_ctx ctx = { .rssi = rssi, .addr = addr };
	bt_data_parse(buf, ad_parse, &ctx);
}

int ble_sensors_init(void)
{
	k_mutex_init(&cache_lock);

	int err = bt_enable(NULL);
	if (err) {
		printk("BLE: enable failed (%d)\n", err);
		return err;
	}

	/*
	 * Critical-alarm receive window, protocol Rev.1 section 3.6:
	 * listen RX_WINDOW_MS (100 ms) every RX_INTERVAL_MS (5 s). BLE scan
	 * parameters are in 0.625 ms units, so the spec values are converted
	 * here rather than hardcoded - previously these constants existed in
	 * app_config.h but nothing used them, and the scan ran 30 ms/1000 ms.
	 */
	struct bt_le_scan_param sp = {
		.type     = BT_LE_SCAN_TYPE_PASSIVE,
		.options  = BT_LE_SCAN_OPT_NONE,
		.interval = BLE_SCAN_UNITS(RX_INTERVAL_MS),
		.window   = BLE_SCAN_UNITS(RX_WINDOW_MS),
	};
	err = bt_le_scan_start(&sp, scan_cb);
	if (err) {
		printk("BLE: scan start failed (%d)\n", err);
		return err;
	}
	printk("BLE: background observer running\n");
	return 0;
}

void ble_sensors_register_alarm_cb(void (*cb)(const struct sw_adv *a, int8_t rssi))
{
	alarm_cb = cb;
}

void ble_sensors_refresh(int window_ms)
{
	/* higher-duty boost (~83%) for a short window */
	struct bt_le_scan_param boost = {
		.type     = BT_LE_SCAN_TYPE_PASSIVE,
		.options  = BT_LE_SCAN_OPT_NONE,
		.interval = 0x0060,
		.window   = 0x0050,
	};
	/* low-power background (~3%) */
	struct bt_le_scan_param bg = {
		.type     = BT_LE_SCAN_TYPE_PASSIVE,
		.options  = BT_LE_SCAN_OPT_NONE,
		.interval = BLE_SCAN_UNITS(RX_INTERVAL_MS),
		.window   = BLE_SCAN_UNITS(RX_WINDOW_MS),
	};

	bt_le_scan_stop();
	if (bt_le_scan_start(&boost, scan_cb) == 0) {
		k_msleep(window_ms);       /* collect stragglers */
	}
	bt_le_scan_stop();
	bt_le_scan_start(&bg, scan_cb);    /* restore low-power scan */
}

int ble_sensors_snapshot(struct sw_node_entry *out, int max)
{
	int count = 0;
	k_mutex_lock(&cache_lock, K_FOREVER);
	for (int i = 0; i < SW_MAX_NODES && count < max; i++) {
		if (cache[i].seen) {
			out[count++] = cache[i];
		}
	}
	k_mutex_unlock(&cache_lock);
	return count;
}

bool ble_sensors_get(uint8_t node_id, struct sw_node_entry *out)
{
	bool seen = false;
	if (node_id < SW_MAX_NODES) {
		k_mutex_lock(&cache_lock, K_FOREVER);
		if (cache[node_id].seen) {
			*out = cache[node_id];
			seen = true;
		}
		k_mutex_unlock(&cache_lock);
	}
	return seen;
}

/* ---- scan control for the central role ---------------------------------
 * The controller cannot scan and initiate a connection simultaneously, so
 * node_link brackets its connect sequence with these. Resume restores the same
 * low-power background parameters used at init.
 */
void ble_sensors_scan_pause(void)
{
	bt_le_scan_stop();
}

void ble_sensors_scan_resume(void)
{
	struct bt_le_scan_param bg = {
		.type     = BT_LE_SCAN_TYPE_PASSIVE,
		.options  = BT_LE_SCAN_OPT_NONE,
		.interval = BLE_SCAN_UNITS(RX_INTERVAL_MS),
		.window   = BLE_SCAN_UNITS(RX_WINDOW_MS),
	};
	(void)bt_le_scan_start(&bg, scan_cb);
}

#if DEBUG_TRACE
void ble_sensors_dump(void)
{
	int64_t now  = k_uptime_get();
	int     up   = 0;

	printk("%s---- NODES (roster %d) ------------------------------------%s",
	       "\n", wagon_node_count(), "\n");

	for (int i = 0; i < wagon_node_count(); i++) {
		const struct wagon_node *w = wagon_node_at(i);
		struct sw_node_entry e;
		bool have;

		k_mutex_lock(&cache_lock, K_FOREVER);
		e    = cache[w->id];
		have = e.seen;
		k_mutex_unlock(&cache_lock);

		if (!have) {
			printk("  %2u %-8s %-8s  MISSING%s",
			       w->id, sw_typ_str(w->type), w->pos, "\n");
			continue;
		}
		up++;

		/*
		 * Age matters as much as presence: a node heard once at boot and
		 * silent since is NOT healthy, but a plain "seen" flag cannot say
		 * so. NODE_STALE_MS is the same limit telem_check_node_health()
		 * uses to raise SENSOR_FAULT, so the console and the uplink agree
		 * about what counts as down.
		 */
		int32_t age_s = (int32_t)((now - e.ts_ms) / 1000);
		const char *st = (now - e.ts_ms) > NODE_STALE_MS ? "STALE" : "ok";

		printk("  %2u %-8s %-8s  %-5s %3ds  rssi %4d  "
		       "val %6d  val2 %6d  batt %3u%%  flags 0x%02X%s",
		       w->id, sw_typ_str(w->type), w->pos, st, age_s, e.rssi,
		       e.data.value, e.data.value2, e.data.batt, e.data.flags,
		       "\n");
	}
	printk("  %d/%d heard%s%s", up, wagon_node_count(), "\n", "\n");
}
#endif
