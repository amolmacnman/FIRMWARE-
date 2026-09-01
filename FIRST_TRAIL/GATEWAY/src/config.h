#ifndef CONFIG_H
#define CONFIG_H
#include <stdint.h>

/*
 * Persisted, server-settable gateway configuration (dn/cmd set_interval /
 * set_threshold / set_gnss).
 *
 * LOW-POWER / LOW-MEMORY design:
 *  - The MASTER copy lives in the external FRAM; the WORKING copy is this one
 *    small SRAM struct (~20 B). The measurement/hot path reads the SRAM copy
 *    only - it NEVER touches FRAM per report, so there is no per-report write
 *    energy.
 *  - FRAM is written ONLY when a value actually changes (config_save), i.e. on
 *    a set_* command - a rare event. FRAM's byte-atomic write means a power
 *    cut mid-write can lose at most this one small record, not the ring buffer.
 *  - Integers only (no floats stored): the impact threshold is milli-g.
 */
struct app_cfg {
	uint32_t magic;
	uint16_t ver;
	uint16_t impact_mg;      /* impact alarm threshold, milli-g        */
	uint32_t moving_s;       /* heartbeat period while moving  (s)     */
	uint32_t idle_s;         /* heartbeat period while stopped (s)     */
	uint16_t gnss_timeout_s; /* max wait for a fix per report  (s)     */
	uint8_t  gnss_enable;    /* 0 = skip fixes (report last-known)     */
	uint8_t  gnss_constel;   /* 0 auto, 1 navic, 2 gps, 3 navic_gps    */
	uint32_t seq;            /* monotonic uplink sequence (persisted)  */
	int8_t   ble_tx_dbm;     /* BLE output power (dn/cmd set_ble_tx)   */
	uint16_t batt_full_mv;   /* LTO pack -> 100 %  (dn/cmd set_batt)   */
	uint16_t batt_empty_mv;  /* LTO pack ->   0 %  (dn/cmd set_batt)   */
} __attribute__((packed));

/*
 * NO rake / group id is stored here, deliberately.
 *
 * An earlier revision added one so the gateway could subscribe to a
 * "smartwagon/v1/grp/<rake>/dn/cmd" broadcast topic. That was wrong on two
 * counts. Protocol Rev.1 s7.1 defines exactly ONE downlink topic,
 * smartwagon/v1/{wgn}/{gw}/dn/cmd, and the cloud's documented wildcards
 * (s7.2) assume position 3 is a wagon number - a literal "grp" there breaks
 * every subscriber built to the spec. And a rake is re-marshalled constantly,
 * so any stored rake id goes stale on the next formation.
 *
 * Group and bulk commands (RDSO s5.1.13) are instead expressed in the PAYLOAD
 * as "scope", and the CLOUD fans them out one publish per wagon on each
 * wagon's own topic. The cloud already knows the current rake composition, so
 * nothing needs to be stored on the wagon - and per-wagon publishing gives
 * delivery confirmation that a broadcast never could.
 */

extern struct app_cfg g_cfg;   /* the live working copy (SRAM) */

/*
 * IDENTITY - read these, never the WAGON_NUMBER macro.
 *
 * Everything that identifies this wagon derives from one string: the MQTT
 * client id, the {wgn} topic key, the BLE isolation group (CRC16) and the
 * per-wagon AES-CCM key (HKDF). It used to be a compile-time #define, which
 * put it inside the firmware image - and an OTA replaces the image, so a
 * fleet-wide update would have overwritten every wagon's identity with
 * whichever one was compiled into the .bin.
 *
 * Holding it in FRAM instead means the image carries no identity at all, so
 * ONE binary serves the whole fleet and group/bulk ota_start (RDSO S5.1.13)
 * work as designed.
 *
 * cfg_gw_id() returns "GW-<wagon>", built once on first call.
 */
/*
 * Load identity from on-chip NVS. MUST be called before anything derives from
 * the wagon number - the BLE isolation group and the AES-CCM key are computed
 * once at startup, and computing them from the compile-time seed because the
 * stored value had not been read yet is silent and total: the gateway would
 * publish to one wagon's topic and be unable to decrypt any node on it.
 *
 * Deliberately separate from config_load(). Identity is needed at the very top
 * of main(); the operational config in FRAM is not needed until the first
 * report, and lives behind an external part that may be absent.
 */
void cfg_identity_load(void);

const char *cfg_wagon(void);
const char *cfg_gw_id(void);

/*
 * Re-provision the wagon number (RDSO S7.18/S7.19) and persist it.
 *
 * Returns 0, or -EINVAL if the string is empty or too long. The caller must
 * expect every derived value to be stale afterwards - the BLE key, the group
 * and the MQTT topics are all computed from this - so the gateway reboots
 * rather than trying to rebuild them in place.
 */
int cfg_set_wagon(const char *w);

/*
 * Roster override. 0 means "use the mask compiled into this image".
 *
 * Kept as an OVERRIDE rather than a copy so the two ways a roster can change
 * stay independent: a firmware update carrying a new default still reaches
 * every wagon that never overrode, while a wagon retro-fitted in service keeps
 * its own answer through that same update. Seeding FRAM from the image at
 * first boot would have made the second impossible.
 */
uint32_t cfg_fitment(void);
int cfg_set_fitment(uint32_t mask);

/*
 * LTO pack state-of-charge map, millivolts (dn/cmd set_batt).
 *
 * Read these rather than the GW_BATT_*_MV macros, which are only the factory
 * seed. Unlike the sub-nodes' flat Li-SOCl2 cell, LTO discharges on a slope, so
 * a voltage->percent map is genuinely meaningful for this pack.
 *
 * cfg_set_batt() returns 0, or -EINVAL if the pair falls outside
 * GW_BATT_MV_MIN..MAX or is separated by less than GW_BATT_MV_SPAN. It persists
 * immediately and takes effect on the next charger read - no reboot, because
 * nothing else is derived from these two values.
 */
/*
 * Gateway BLE output power, dBm (dn/cmd set_ble_tx).
 *
 * Applied to each CONNECTION as it comes up - the gateway does not advertise,
 * so a connection is the only time it transmits. Takes effect on the next node
 * link, no reboot.
 */
int8_t cfg_ble_tx_dbm(void);
int    cfg_set_ble_tx(int8_t dbm);

uint16_t cfg_batt_full_mv(void);
uint16_t cfg_batt_empty_mv(void);
int      cfg_set_batt(uint16_t full_mv, uint16_t empty_mv);

/* Boot: load FRAM -> g_cfg; seed compile-time defaults if blank/invalid. */
void config_load(void);

/* Persist g_cfg -> FRAM. Call ONLY after a value has changed. */
void config_save(void);

/* Increment the persistent uplink sequence, write just that field to FRAM,
 * and return the new value. Survives resets so the cloud's (gw,seq) de-dup and
 * ordering keep working - one small 4-byte FRAM write per report (low power). */
uint32_t config_next_seq(void);

#endif /* CONFIG_H */
