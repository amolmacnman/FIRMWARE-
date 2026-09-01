/*
 * Gateway-level alarm and event detection - the parts of protocol Rev.1
 * sections 3.2 and 4.2 that are NOT a direct relay of a sub-node's ALARM flag.
 *
 * A sub-node reports a raw condition (over-temp, door open, impact). Deciding
 * whether that is an ALARM, an EVENT, or nothing is the gateway's job, because
 * only the gateway knows motion state, power posture and the previous value.
 * That decision needs edge detection - "changed since last cycle" - so the
 * previous state has to live somewhere. It lives here rather than being
 * scattered through main.c.
 *
 * Everything here is evaluated once per report cycle from gwalarm_eval(),
 * except the derailment check which runs on the impact wake path.
 */
#ifndef GWALARM_H
#define GWALARM_H

#include <stdint.h>
#include <stdbool.h>
#include "sensor_proto.h"

/* Load persisted edge state. Call once at boot, after config_load(). */
void gwalarm_init(void);

/*
 * Evaluate every gateway-level condition against the current node cache and
 * power state, publishing any alarms/events that fire. Call once per report
 * cycle, after the sensor snapshot is fresh.
 *
 * Emits, per protocol Rev.1:
 *   alarms  FLAT_WHEEL, OVERLOAD, LOW_BATTERY, NODE_LOW_BATTERY
 *   events  DOOR_OPEN/DOOR_CLOSE, HANDBRAKE_APPLIED/_RELEASED, LOAD_CHANGE,
 *           BAND_CHANGE, CHARGE_START/_STOP, SRC_SWITCH, GEOFENCE_ENTER/_EXIT
 */
void gwalarm_eval(bool moving);

/*
 * Impact-wake classifier. A large shock while MOVING is a derailment
 * candidate (section 5.3.3); a smaller one, or any shock while stopped, is a plain
 * IMPACT. Publishes the appropriate alarm and returns the code used.
 */
const char *gwalarm_impact(double g, bool moving);

/* Called after the link comes back up following a failed cycle -> CONN_RESTORED. */
void gwalarm_link_up(bool was_offline);

/*
 * Condition band for one node: 'G', 'Y' or 'R' (RDSO s7.10), or 'G' for a node
 * that has never been heard.
 *
 * Exposed so the HEARTBEAT can report the same band the BAND_CHANGE event
 * reports. It used to publish a hardcoded "G" for every asset, so a bearing
 * could cross into Red, raise a correct BAND_CHANGE event, and still be shown
 * as Green by the very next heartbeat - the periodic record that dashboards
 * poll and that s7.12 trends. The two uplinks contradicting each other is worse
 * than either being absent.
 *
 * The band is recomputed in gwalarm_eval(); this only reads the stored value,
 * so call it after the evaluation for the current cycle.
 */
/*
 * RDSO s7.10/s7.11 condition band for one node: 'G', 'Y', 'R', or '?' when the
 * node has never been heard and there is nothing to band.
 *
 * Callers MUST encode '?' as JSON null rather than emitting it as a band value:
 * s7.11 defines exactly three bands, and a fourth letter in that field would be
 * an undocumented protocol extension. null is unambiguous to any consumer.
 */
char gwalarm_band(uint8_t node_id);

/*
 * RDSO s7.10/s7.11 WHEEL condition band for one bearing node: 'G', 'Y', 'R', or
 * '?' when the node has not been heard or reported no vibration index.
 *
 * Derived from the vibration index (advert value2), NOT from bearing
 * temperature - a wheel flat shows as a vibration signature while its bearing
 * may be perfectly cool. Same '?' -> JSON null contract as gwalarm_band().
 */
char gwalarm_wheel_band(uint8_t node_id);

#endif /* GWALARM_H */
