#ifndef EC200_H
#define EC200_H

#include <stdbool.h>
#include <stdint.h>

/* Initialise the UART to the EC200 modem. */
void ec200_init(void);

/* Bring the modem online: AT ready, SIM ready, PDP context activated.
 * Returns 0 on success, negative on failure. */
/* Bring the modem up using all_hw_test's exact polled sequence. */
/* Polled AT check. Never touches PWRKEY. */
bool ec200_at_ok(int tries);

bool ec200_bringup(void);

int ec200_modem_up(void);

/* Bench probe: send AT and read the reply by POLLING, bypassing the RX
 * interrupt and its ring buffer. Tells a dead ISR apart from a dead modem. */
int ec200_at_polled(void);

/* Open + connect the MQTT session to the configured broker.
 * Returns 0 on success. */
int ec200_mqtt_up(void);

/* Publish a payload string to a topic (QoS0). Returns 0 on success. */
int ec200_mqtt_publish(const char *topic, const char *payload);

/* Graceful MQTT disconnect + modem power-down (per-wake power saving). */
void ec200_disconnect(void);

/*
 * True only if the broker session is CURRENTLY connected.
 *
 * Asks the modem (AT+QMTCONN? -> +QMTCONN: 0,3) rather than trusting a cached
 * flag: the session can drop underneath us at any time - lost registration, a
 * broker-side keepalive timeout, the carrier tearing down the PDP context - and
 * none of those tell the application. A stale "connected" belief would send
 * every publish into a dead socket and silently lose the data.
 *
 * Used by the bench keep-alive path to decide whether the full attach sequence
 * can be skipped this cycle.
 */
bool ec200_mqtt_is_up(void);

/*
 * Download a firmware image over HTTP(S) using the EC200's QHTTP AT flow
 * (QHTTPURL -> QHTTPGET -> QHTTPREAD). Each received chunk is passed to
 * `sink(ctx, data, len)`; return non-zero from sink to abort. Returns the total
 * content length on success, negative on error. ota.c streams straight from
 * this into the MCUboot secondary slot, so the whole image never sits in RAM.
 *
 * HTTPS needs a CA cert provisioned on the modem (AT+QFUPL + AT+QSSLCFG) or a
 * relaxed seclevel; the image is MCUboot-signed regardless, so integrity is
 * guaranteed at swap time even over plain HTTP.
 */
typedef int (*ec200_sink_fn)(void *ctx, const uint8_t *data, int len);
int ec200_http_download(const char *url, ec200_sink_fn sink, void *ctx);

/*
 * Discover the APN this SIM is actually using, and print SIM identifiers.
 *
 * The APN is NOT encoded in the ICCID printed on the card - it is provisioned
 * against the operator account. On LTE the network usually activates the
 * default bearer at attach using the APN from the SIM profile, so after
 * AT+CGATT the modem can simply be asked what it got:
 *
 *   AT+CGDCONT?  ->  +CGDCONT: 1,"IP","<apn>","<ip>",0,0
 *
 * Copies the CID-1 APN into `out`. Returns 0 on success, <0 if the modem does
 * not answer, the SIM is not READY, or the network never attaches (in which
 * case there is no APN to report yet).
 *
 * Also prints IMSI and ICCID - quote those to the operator to be told the APN
 * for the account if the network does not supply one.
 */
int ec200_query_apn(char *out, size_t n);

/*
 * True if the command most recently returned by ec200_mqtt_poll_cmd() arrived
 * on the fleet-wide topic (smartwagon/v1/all/dn/cmd) rather than this wagon's
 * own. Valid only until the next poll. Used to force fleet behaviour - notably
 * the OTA download stagger - even when the sender omitted "scope".
 */
bool ec200_last_cmd_was_bulk(void);

#endif /* EC200_H */
