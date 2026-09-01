/*
 * Gateway configuration store (FRAM-backed, server-settable).
 * See config.h for the low-power / low-memory rationale.
 *
 * FRAM map:  offset 0  -> storage.c ring-buffer meta (16 B)
 *            offset 64 -> this app_cfg (leave a gap for future growth)
 */
#include <zephyr/kernel.h>
#include <zephyr/drivers/eeprom.h>
#include <zephyr/settings/settings.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stddef.h>
#include "config.h"
#include "sensor_proto.h"   /* sw_txpwr_sane */
#include "app_config.h"

#define CFG_MAGIC     0x53574347u   /* 'SWCG' */
/*
 * CFG_VER gates whether a stored struct is accepted. config_load() re-seeds
 * from the compile-time defaults whenever the stored version does not match,
 * which is the ONLY mechanism by which a changed default ever reaches a board
 * that has already booted once.
 *
 * 3: bumped so the DEBUG_TRACE bench cadence actually takes effect. Until the
 *    memory rail was raised early enough for FRAM to probe, every boot fell
 *    through to seed_defaults() and this never mattered - now that FRAM works,
 *    a stored v2 struct would pin the old 10 min cadence forever.
 * 2: added the persistent seq field.
 */
/*
 * 4: added wagon[16]. Bumping this deliberately discards any v3 record - the
 * struct layout changed, and a stale one would be read with every field
 * shifted. Re-seeding from the compile-time defaults is the safe outcome.
 */
/*
 * 9: the struct gained batt_full_mv / batt_empty_mv (dn/cmd set_batt) and
 * ble_tx_dbm (dn/cmd set_ble_tx). It grew twice without this number moving -
 * a stored older record would have been ACCEPTED and its trailing fields read
 * as whatever the neighbouring bytes happened to be. Gating on this is the only
 * thing that stops a layout change being silently misinterpreted.
 */
#define CFG_VER       9
#define CFG_FRAM_OFF  64            /* after the storage ring meta at offset 0 */

static const struct device *const fram = DEVICE_DT_GET(DT_NODELABEL(fram));

struct app_cfg g_cfg;

static void seed_defaults(void)
{
	g_cfg.magic          = CFG_MAGIC;
	g_cfg.ver            = CFG_VER;
	g_cfg.impact_mg      = (uint16_t)(IMPACT_G_THRESH * 1000.0);
	g_cfg.moving_s       = INTERVAL_MOVING_MS / 1000u;
	g_cfg.idle_s         = INTERVAL_IDLE_MS   / 1000u;
	/*
	 * Fix timeout. 90 s in production gives a cold start a fair chance;
	 * 40 s on the bench, where the receiver is usually indoors and will not
	 * fix at all, so every extra second is a second not spent testing the
	 * modem and the uplink behind it.
	 */
#if DEBUG_TRACE
	g_cfg.gnss_timeout_s = 40;
#else
	g_cfg.gnss_timeout_s = 90;
#endif
	g_cfg.gnss_enable    = 1;
	g_cfg.gnss_constel   = 3;        /* navic_gps */
	g_cfg.seq            = 0;

	/*
	 * These were missing, and only the zero-fallbacks in cfg_batt_full_mv()
	 * / cfg_batt_empty_mv() hid it: a freshly seeded config left both at 0,
	 * the accessors substituted the macros, and everything looked correct.
	 * That made a defence-in-depth check the ONLY thing keeping the SoC map
	 * alive - and it would have stopped working the moment someone trusted
	 * g_cfg directly instead of the accessor.
	 */
	g_cfg.batt_full_mv   = GW_BATT_FULL_MV;
	g_cfg.batt_empty_mv  = GW_BATT_EMPTY_MV;
	g_cfg.ble_tx_dbm     = GW_BLE_TX_DBM;
	/*
	 * FACTORY SEED ONLY. Used when FRAM holds nothing valid for this
	 * CFG_VER - i.e. a virgin board, or one whose struct layout changed.
	 * Once stored, FRAM is authoritative and a later image carrying a
	 * different default will NOT overwrite it. That is what lets one
	 * binary update a whole fleet without touching identity.
	 */
}

/*
 * IDENTITY - on-chip NVS, not the FRAM alongside app_cfg.
 *
 * app_cfg is in FRAM because config_next_seq() writes it every report, where
 * FRAM's endurance and byte-atomic write earn their place. Identity is written
 * once at commissioning, so it gains nothing from that and inherits a
 * dependency on an external part sitting behind a switched rail. A gateway
 * whose FRAM is absent or whose VCC_MEM has failed must still know which wagon
 * it is - that is the last thing it should lose, not the first.
 */
static char     s_wagon[16];
static uint32_t s_fit;

static int id_set(const char *name, size_t len, settings_read_cb read_cb,
		  void *cb_arg)
{
	const char *next;

	if (settings_name_steq(name, "wagon", &next) && !next) {
		int rc = read_cb(cb_arg, s_wagon, sizeof(s_wagon) - 1);

		s_wagon[rc > 0 ? rc : 0] = '\0';
		return 0;
	}
	if (settings_name_steq(name, "fit", &next) && !next) {
		return read_cb(cb_arg, &s_fit, sizeof(s_fit)) > 0 ? 0 : -EINVAL;
	}
	return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(swid, "sw/id", NULL, id_set, NULL, NULL);

void cfg_identity_load(void)
{
	(void)settings_subsys_init();
	(void)settings_load_subtree("sw/id");

	printk("identity: wagon %s (%s), fitment %s\n",
	       cfg_wagon(), s_wagon[0] ? "provisioned" : "COMPILE-TIME SEED",
	       s_fit ? "overridden" : "from image");
}

const char *cfg_wagon(void)
{
	/*
	 * Fall back to the compile-time value when nothing is stored. An empty
	 * wagon number would build the topic "smartwagon/v1//GW-/up" and a
	 * client id of "GW-", which every unprovisioned gateway would share -
	 * and brokers disconnect the older session on a duplicate id, so they
	 * would kick each other off in a loop. Behaving like the old build is
	 * strictly better than a fleet-wide collision.
	 */
	return s_wagon[0] ? s_wagon : WAGON_NUMBER;
}

const char *cfg_gw_id(void)
{
	static char id[20];

	if (id[0] == '\0' || strncmp(id + 3, cfg_wagon(), sizeof(id) - 4)) {
		snprintf(id, sizeof(id), "GW-%s", cfg_wagon());
	}
	return id;
}

uint32_t cfg_fitment(void)
{
	return s_fit;
}

int cfg_set_fitment(uint32_t mask)
{
	/*
	 * Bits above 18 are refused: node ids run 0..18, so a high bit is a
	 * typo or a mis-encoded value, and accepting it would leave the gateway
	 * in permanent SENSOR_FAULT waiting for a node that cannot exist.
	 * 0 is legal and means "go back to following the image".
	 */
	if (mask & ~0x7FFFFu) {
		return -EINVAL;
	}
	s_fit = mask;
	settings_save_one("sw/id/fit", &s_fit, sizeof(s_fit));
	printk("config: fitment override -> 0x%05X%s\n", mask,
	       mask ? "" : " (follow image)");
	return 0;
}

int cfg_set_wagon(const char *w)
{
	if (!w || !w[0] || strlen(w) >= sizeof(s_wagon)) {
		return -EINVAL;
	}
	strncpy(s_wagon, w, sizeof(s_wagon) - 1);
	s_wagon[sizeof(s_wagon) - 1] = '\0';
	settings_save_one("sw/id/wagon", s_wagon, strlen(s_wagon));
	printk("config: wagon number set to %s\n", s_wagon);
	return 0;
}

uint32_t config_next_seq(void)
{
	g_cfg.seq++;
	if (device_is_ready(fram)) {
		/* write ONLY the 4-byte seq field, not the whole struct */
		(void)eeprom_write(fram,
				   CFG_FRAM_OFF + offsetof(struct app_cfg, seq),
				   &g_cfg.seq, sizeof(g_cfg.seq));
	}
	return g_cfg.seq;
}

int8_t cfg_ble_tx_dbm(void)
{
	return g_cfg.ble_tx_dbm;
}

int cfg_set_ble_tx(int8_t dbm)
{
	if (!sw_txpwr_sane(dbm)) {
		return -EINVAL;
	}
	if (dbm == g_cfg.ble_tx_dbm) {
		return 0;              /* unchanged - do not burn a FRAM write */
	}
	g_cfg.ble_tx_dbm = dbm;
	config_save();
	printk("config: gateway BLE TX -> %d dBm (next node link)\n", dbm);
	return 0;
}

uint16_t cfg_batt_full_mv(void)
{
	/*
	 * Fall back to the seed if the stored value is zero.
	 *
	 * A zeroed pair would make soc_from_vbat() divide by zero, and the one
	 * way to reach that state is a config record written before these fields
	 * existed. CFG_VER should already have re-seeded such a record, so this
	 * is the second line of defence rather than the first - but the cost of
	 * being wrong is a divide-by-zero on the reporting path.
	 */
	return g_cfg.batt_full_mv ? g_cfg.batt_full_mv : GW_BATT_FULL_MV;
}

uint16_t cfg_batt_empty_mv(void)
{
	return g_cfg.batt_empty_mv ? g_cfg.batt_empty_mv : GW_BATT_EMPTY_MV;
}

int cfg_set_batt(uint16_t full_mv, uint16_t empty_mv)
{
	if (full_mv < GW_BATT_MV_MIN || full_mv > GW_BATT_MV_MAX ||
	    empty_mv < GW_BATT_MV_MIN || empty_mv > GW_BATT_MV_MAX) {
		return -EINVAL;
	}
	if ((int)full_mv - (int)empty_mv < GW_BATT_MV_SPAN) {
		return -EINVAL;
	}

	if (full_mv == g_cfg.batt_full_mv && empty_mv == g_cfg.batt_empty_mv) {
		return 0;              /* unchanged - do not burn a FRAM write */
	}

	g_cfg.batt_full_mv  = full_mv;
	g_cfg.batt_empty_mv = empty_mv;
	config_save();
	printk("config: LTO map set - full %u mV, empty %u mV\n",
	       full_mv, empty_mv);
	return 0;
}

void config_load(void)
{
	if (!device_is_ready(fram) ||
	    eeprom_read(fram, CFG_FRAM_OFF, &g_cfg, sizeof(g_cfg)) != 0 ||
	    g_cfg.magic != CFG_MAGIC || g_cfg.ver != CFG_VER) {
		seed_defaults();
		config_save();
		printk("config: seeded defaults\n");
	} else {
		printk("config: loaded (moving=%us idle=%us impact=%umg gnss=%u)\n",
		       g_cfg.moving_s, g_cfg.idle_s, g_cfg.impact_mg,
		       g_cfg.gnss_enable);
	}

#if DEBUG_TRACE
	/*
	 * Warn, but do NOT override.
	 *
	 * An earlier version forced moving_s/idle_s here on every boot. That made
	 * the cadence impossible to test over the air: a set_interval downlink
	 * would apply, persist to FRAM, and then be silently reverted on the next
	 * reboot - so a working command looked broken.
	 *
	 * The bench values reach a board through seed_defaults() instead, which
	 * runs only when FRAM holds nothing valid for this CFG_VER. After that the
	 * stored value wins, exactly as in production, and set_interval behaves
	 * identically to the way it will in the field.
	 */
	printk("\n"
	       "  ****************************************************\n"
	       "  *  DEBUG_TRACE = 1  -  BENCH BUILD, DO NOT SHIP     *\n"
	       "  *  cadence now moving=%-4us idle=%-4us               *\n"
	       "  *  RDSO s7.4 requires 10 min; s5.1.20 battery life  *\n"
	       "  *  is calculated from it. Set DEBUG_TRACE 0.        *\n"
	       "  ****************************************************\n\n",
	       g_cfg.moving_s, g_cfg.idle_s);
#endif
}

void config_save(void)
{
	if (device_is_ready(fram)) {
		(void)eeprom_write(fram, CFG_FRAM_OFF, &g_cfg, sizeof(g_cfg));
	}
}
