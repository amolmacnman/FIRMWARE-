#ifndef GNSS_H
#define GNSS_H

#include <stdbool.h>
#include <stdint.h>

/* Latest GNSS fix, decoded from the LC29H RMC sentence. */
struct gnss_fix {
	bool    valid;          /* true when RMC status = A (has fix)      */
	double  lat_deg;        /* decimal degrees, +N / -S                */
	double  lon_deg;        /* decimal degrees, +E / -W                */
	double  speed_kmh;      /* ground speed in km/h                    */
	double  course_deg;     /* course over ground, degrees true        */
	char    utc_iso[32];    /* "2026-08-05T12:00:00Z" ("" if unknown)  */
	int64_t updated_ms;     /* k_uptime_get() when last RMC was parsed */

	/* From GGA. RDSO s7.17 requires position accuracy to be REPORTED, so cep
	 * must be derived from the engine, not asserted as a constant. */
	uint8_t fix_q;          /* GGA quality: 0 none, 1 GPS, 2 DGPS      */
	uint8_t nsat;           /* satellites used in the fix              */
	float   hdop;           /* horizontal dilution of precision        */
};

/*
 * CEP50 in metres, derived from HDOP (RDSO s7.17: "CEP50 less than 5m").
 * CEP50 ~ HDOP x UERE, UERE ~2.5 m for a modern multi-GNSS receiver in the
 * open. Returns 99 when there is no usable fix so the cloud can distinguish a
 * poor fix from a good one, instead of trusting a hardcoded value.
 */
uint8_t gnss_cep_m(const struct gnss_fix *f);

/*
 * Constellations that actually produced recent fixes, as a JSON array e.g.
 * ["gps","galileo"]. Protocol Rev.1 s2.3 loc.sys - derived from NMEA talker IDs
 * rather than asserted, so it reports provenance instead of intent. Returns the
 * number of bytes written.
 */
int gnss_sys_json(char *out, size_t n);

/* Start reading NMEA from the LC29H on its UART. */
void gnss_init(void);

/* Copy the most recent fix into *out (thread-safe). */
void gnss_get(struct gnss_fix *out);

/* Current motion state (with hysteresis), thread-safe. */
bool gnss_is_moving(void);

/*
 * Apply the constellation selection to the LC29H (0 auto, 1 navic, 2 gps,
 * 3 navic_gps) via PAIR066 + PAIR513 (persisted to module flash). Call with the
 * GNSS rail powered. NOTE: NavIC is not supported by the LC29H per its protocol
 * spec - the navic options fall back to multi-GNSS (see gnss.c).
 */
void gnss_set_constellation(uint8_t constel);

/*
 * Block until a FRESH valid fix is parsed, or timeout. Returns 0 on a new fix,
 * -EAGAIN on timeout. The caller sleeps (no polling) while the GNSS powers up
 * and locks. Call gnss_power_on() first.
 */
int gnss_wait_fix(int timeout_ms);

/*
 * Register a callback that fires ONCE whenever the motion state flips
 * (stopped->moving or moving->stopped). Called from the GNSS thread; keep it
 * short (e.g. just post an event). This is what makes reporting event-driven
 * instead of polled.
 */
void gnss_register_motion_cb(void (*cb)(bool moving));

#endif /* GNSS_H */
