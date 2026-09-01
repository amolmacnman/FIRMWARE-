#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include "sensor_proto.h"

/* Build + publish protocol messages (envelope + topic per §7). */
void telem_init(void);

int  telem_pub_heartbeat(const char *mode);                 /* up/hb   */
int  telem_pub_node_alarm(const struct sw_adv *n, int moving); /* up/alarm */
int  telem_pub_impact_alarm(double g);                      /* up/alarm */
int  telem_pub_alarm(const char *code, const char *sev, const char *node_ser,
		    double val, const char *unit, double thr, const char *st);
void telem_node_serial(char *o, size_t n, uint8_t type, uint8_t id);

int  telem_pub_event(const char *code, const char *from,
		     const char *to, const char *trip);      /* up/event, node:null */
/* Same, but stamped with the sensor that caused it. Use for NODE-LEVEL events
 * (door, bearing band, load, handbrake); chassis-level events keep node:null. */
int  telem_pub_event_node(const char *code, const char *from, const char *to,
			  const char *trip, uint8_t node_type, uint8_t node_id);
int  telem_pub_response(const char *cid, const char *cmd,
			const char *res, const char *ec_or_null,
			const char *pl_json);                   /* up/resp  */

uint32_t telem_epoch(void);   /* current UTC epoch (GNSS or software clock) */

/* Set the software clock (used by the time_sync command and internally by the
 * GNSS fix). epoch = UTC seconds. */
void telem_set_time(uint32_t epoch);

/* Replay buffered records (oldest-first) after a reconnect. */
void telem_flush_backlog(void);

/* Check every provisioned node's liveness; emit SENSOR_FAULT set/clear on
 * a node going down / recovering. Call once per scheduled report. */
void telem_check_node_health(void);

#endif /* TELEMETRY_H */
