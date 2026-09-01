#ifndef MODEM_CMD_H
#define MODEM_CMD_H
#include <stddef.h>

/*
 * Downlink command reception over the EC200.
 * Returns 1 and fills `out` with the command JSON if one was queued on
 * dn/cmd, 0 if none within timeout_ms, negative on error.
 */
/* Let a command the broker pushed on the back of the SUBACK land before the
 * birth publish runs, so it can be answered in the same cycle. Call once,
 * right after a successful AT+QMTSUB. */
void ec200_stash_pending_cmd(void);

int ec200_mqtt_poll_cmd(char *out, size_t n, int timeout_ms);

#endif /* MODEM_CMD_H */
