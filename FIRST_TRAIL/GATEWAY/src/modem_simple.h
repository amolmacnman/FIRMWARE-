#ifndef MODEM_SIMPLE_H
#define MODEM_SIMPLE_H

#include <stdbool.h>

/*
 * The EC200U path copied from all_hw_test - the only code that has ever made
 * this modem attach and publish on this board. See modem_simple.c for what
 * differs between the two environments and why each difference is handled
 * explicitly rather than assumed away.
 *
 * Brings the module up on first use, attaches, opens the broker, and publishes.
 * Later calls reuse the session, so a report is a single QMTPUB.
 */
int  modem_simple_publish(const char *topic, const char *payload);

/*
 * Bring the modem up and connect to the broker without publishing anything.
 * Returns 0 when the session is good.
 *
 * This is the gateway's authority on whether it is online. ec200.c cannot be:
 * its AT path has never received a byte on this board, so a link check through
 * it reported "offline" while this module was connected and publishing.
 */
int modem_simple_up(void);

/* Publish with the MQTT retain flag set. Used for the status topic, which a
 * subscriber must be able to read between the gateway's brief online windows. */
int  modem_simple_publish_retain(const char *topic, const char *payload);

/* Mark the broker session closed (called after the modem is powered down). */
void modem_simple_session_closed(void);

/*
 * Last measured signal strength in dBm, or 0 if it could not be measured.
 *
 * 0 is not a legal RSSI, so it cannot be mistaken for a reading - which is the
 * whole point. The heartbeat used to carry a hardcoded -71 that looked like a
 * healthy link regardless of what the radio was actually doing.
 */
int modem_simple_rssi_dbm(void);

/* Serving cell id, or "" when the network did not report one. */
const char *modem_simple_cell(void);

/* True once the broker session is established. */
bool modem_simple_is_up(void);

#endif /* MODEM_SIMPLE_H */
