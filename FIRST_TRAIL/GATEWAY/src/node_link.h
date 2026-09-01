/*
 * Gateway -> sub-node BLE link (the second half of the downlink chain).
 *
 *   server --MQTT--> gateway --BLE--> sub-node
 *
 * A node-targeted command arrives over MQTT long before the target node is
 * reachable: each node opens a connectable window for only ~4 s, roughly every
 * 10 minutes. So a command is QUEUED here and delivered opportunistically the
 * next time that node's window is seen by the observer that is already
 * scanning continuously - no extra radio time, no polling.
 *
 * The MQTT response is therefore "queued", not "applied". Confirmation that a
 * node accepted a setting comes later, on its next uplink advert.
 */
#ifndef NODE_LINK_H
#define NODE_LINK_H

#include <stdint.h>
#include <stdbool.h>

/* Register the window callback and load persisted downlink counters. */
int node_link_init(void);

/*
 * Queue a threshold change for one node. Delivered when that node next opens
 * its window. Returns 0 if queued, <0 if node_id is out of range.
 * Queuing again for the same node replaces the pending command (last wins) -
 * there is ONE slot per node, not a queue, so two different thresholds for the
 * same node must be sent far enough apart for the first to be delivered.
 *
 * `cmd` selects WHICH threshold:
 *   SW_DN_SET_THRESHOLD  primary sensor + hysteresis   (every node type)
 *   SW_DN_SET_IMPACT     BMA400 tamper limit, milli-g  (door + tank only)
 *   SW_DN_SET_VIB        vibration hint index          (bearing only)
 *
 * The gateway does not filter by capability - it queues whatever it is asked
 * to. A node that lacks the sensor refuses the frame itself, which keeps the
 * capability map in one place (the node) instead of duplicated here.
 */
int node_link_queue_cmd(uint8_t node_id, uint8_t node_type, uint8_t cmd,
			int16_t threshold, int16_t hyst);

/*
 * As above, but also sets the frame's `rsvd` byte.
 *
 * Only SW_DN_SET_BATTCAL uses it, to say WHICH gauge constant the value is for -
 * seven settings sharing one opcode, so the 6-byte payload and its CCM buffers
 * stay unchanged. Every other command leaves rsvd at 0, which is exactly what
 * the 5-argument form above passes.
 */
int node_link_queue_cmd_sel(uint8_t node_id, uint8_t node_type, uint8_t cmd,
			    uint8_t sel, int16_t threshold, int16_t hyst);

/* True if a command is still waiting for this node (for get_status). */
bool node_link_pending(uint8_t node_id);

/* Number of nodes with a command still queued. */
int  node_link_pending_count(void);

#endif /* NODE_LINK_H */
