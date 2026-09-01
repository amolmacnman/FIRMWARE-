#ifndef BLE_SENSORS_H
#define BLE_SENSORS_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/bluetooth/addr.h>
#include "sensor_proto.h"

#define SW_MAX_NODES 20

struct sw_node_entry {
	bool           seen;
	struct sw_adv  data;
	int8_t         rssi;
	int64_t        ts_ms;      /* uptime when last heard */
};

/* Set THIS wagon's group id BEFORE init. Adverts from other groups are
 * discarded so neighbouring wagons never contaminate this wagon's data. */
void ble_sensors_set_group(uint16_t wgn_group);

/* Start BLE, begin the low-power background observer. Call once at boot. */
int  ble_sensors_init(void);

/* Register a callback fired when a node broadcasts with the ALARM flag. */
void ble_sensors_register_alarm_cb(void (*cb)(const struct sw_adv *a, int8_t rssi));

/* Register a callback fired when a sub-node opens its CONNECTABLE config/DFU
 * window. node_link uses this to connect and deliver a queued command; the
 * gateway is already scanning continuously, so no extra radio time is needed. */
void ble_sensors_register_window_cb(void (*cb)(uint8_t node_id, uint8_t node_type,
					       const bt_addr_le_t *addr));

/* Pause/resume the background observer around a central connection (the
 * controller cannot scan and initiate at the same time). */
void ble_sensors_scan_pause(void);
void ble_sensors_scan_resume(void);

/* Copy the current cache (one entry per node_id). Returns count of seen nodes. */
int  ble_sensors_snapshot(struct sw_node_entry *out, int max);

/*
 * Print the full expected roster to the console: every node this wagon should
 * have, whether it has been heard, how long ago, and its last reading.
 *
 * Deliberately driven by WAGON_NODES rather than by the receive cache, so a
 * node that has NEVER been heard still appears - as a MISSING row. A dump built
 * from the cache alone would silently omit exactly the nodes worth worrying
 * about.
 *
 * Bench aid; compiled out when DEBUG_TRACE is 0.
 */
void ble_sensors_dump(void);

/* Look up one node by id. Returns true if it has ever been heard. */
bool ble_sensors_get(uint8_t node_id, struct sw_node_entry *out);

/*
 * Briefly raise the scan duty to catch stragglers before a heartbeat, then
 * restore the low-power background scan. Called only at report time (every
 * 10 min / 12 h) so the extra RX energy is negligible. Blocks for window_ms.
 */
void ble_sensors_refresh(int window_ms);

#endif /* BLE_SENSORS_H */
