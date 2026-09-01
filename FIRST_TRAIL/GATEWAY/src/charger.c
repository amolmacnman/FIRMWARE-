/*
 * TI BQ25798 (BQ25798RQMR) charger driver - battery telemetry for the gateway.
 *
 * The BQ25798 has a built-in 16-bit ADC that measures VBAT, VSYS, VBUS, IBAT,
 * etc. We drive it in ONE-SHOT mode (ADC_EN self-clears after one round) so it
 * costs power only when we actually sample, and we cache the result so a burst
 * of uplinks in one wake doesn't re-trigger the I2C + conversion every message.
 *
 * Bus: i2c21 (shared with BMA400 + SHT40).  Address: 0x6B (fixed).
 *
 * REGISTER MAP (per the BQ25798 datasheet - verify against your silicon rev):
 *   0x1C  Charger_Status_1 : CHG_STAT in bits [7:5]
 *   0x2E  ADC_Control      : bit7 ADC_EN, bit6 ADC_RATE (1=one-shot)
 *   0x31/32 IBUS_ADC   0x33/34 IBAT_ADC (signed, 1 mA/LSB)
 *   0x35/36 VBUS_ADC   0x3B/3C VBAT_ADC   0x3D/3E VSYS_ADC  (all 1 mV/LSB)
 */
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include "charger.h"
#include "app_config.h"     /* GW_BATT_FULL_MV / GW_BATT_EMPTY_MV (seeds) */
#include "config.h"         /* cfg_batt_full_mv() / cfg_batt_empty_mv()   */

#define BQ_ADDR         0x6B
#define REG_CHG_STAT1   0x1C
#define REG_PART_INFO   0x48   /* PN[5:3] = 0b011 on the BQ25798 */
#define REG_ADC_CTRL    0x2E
#define REG_IBAT_ADC    0x33
#define REG_VBUS_ADC    0x35
#define REG_VBAT_ADC    0x3B
#define REG_VSYS_ADC    0x3D

#define ADC_EN          0x80
#define ADC_ONESHOT     0x40     /* ADC_RATE=1 -> single conversion round     */

#define REFRESH_MS      20000    /* re-read the charger at most every 20 s    */

static const struct device *const i2c = DEVICE_DT_GET(DT_NODELABEL(i2c21));

static struct charger_status cache;
static int64_t cache_ts;         /* 0 = never read                            */

static int rd16(uint8_t reg, uint16_t *v)
{
	uint8_t b[2];
	int rc = i2c_burst_read(i2c, BQ_ADDR, reg, b, 2);
	if (rc) {
		return rc;
	}
	*v = ((uint16_t)b[0] << 8) | b[1];
	return 0;
}

/* Li-titanate (LTO) has a sloped discharge, so voltage->% is usable (unlike the
 * flat Li-SOCl2 subnode cell). Linear between the pack's empty/full points. */
static uint8_t soc_from_vbat(uint16_t mv)
{
	/* Runtime map, not the compile-time seed: both ends are settable with
	 * dn/cmd set_batt so a pack can be characterised in service without a
	 * firmware build. cfg_set_batt() guarantees full - empty >=
	 * GW_BATT_MV_SPAN, so the divisor below is never zero. */
	uint16_t full  = cfg_batt_full_mv();
	uint16_t empty = cfg_batt_empty_mv();

	if (mv >= full) {
		return 100;
	}
	if (mv <= empty) {
		return 0;
	}
	return (uint8_t)(((uint32_t)(mv - empty) * 100) / (full - empty));
}

int charger_init(void)
{
	if (!device_is_ready(i2c)) {
		printk("charger: i2c21 not ready\n");
		return -ENODEV;
	}

	/*
	 * Read PART_INFO so a missing or mute charger is reported AT BOOT.
	 *
	 * device_is_ready() only says the nRF TWIM controller came up; it says
	 * nothing about whether anything answers at 0x6B. Without this the
	 * gateway boots silently and then publishes battery telemetry assembled
	 * from failed reads - worse than no telemetry, because an operator has no
	 * way to tell a flat wagon from a broken charger.
	 *
	 * PN occupies bits 5:3 and reads 0b011 on the BQ25798. The remaining bits
	 * are silicon revision and are deliberately not checked.
	 */
	uint8_t pi = 0;
	int rc = i2c_reg_read_byte(i2c, BQ_ADDR, REG_PART_INFO, &pi);

	if (rc != 0) {
		printk("charger: BQ25798 not responding at 0x%02X (%d)\n",
		       BQ_ADDR, rc);
		return -ENODEV;
	}
	if (((pi >> 3) & 0x07) != 0x03) {
		printk("charger: unexpected part id 0x%02X at 0x%02X\n",
		       pi, BQ_ADDR);
		return -ENODEV;
	}
	printk("charger: BQ25798 ready\n");
	return 0;
}

int charger_read(struct charger_status *s)
{
	int64_t now = k_uptime_get();
	if (cache_ts != 0 && (now - cache_ts) < REFRESH_MS) {
		*s = cache;                 /* fresh enough - reuse */
		return cache.valid ? 0 : -EIO;
	}

	struct charger_status r = { 0 };

	/* Kick a one-shot ADC round, then wait for it to finish (~24 ms at the
	 * default resolution; 50 ms is a safe margin). */
	if (i2c_reg_write_byte(i2c, BQ_ADDR, REG_ADC_CTRL, ADC_EN | ADC_ONESHOT) != 0) {
		r.valid = false;
		cache = r; cache_ts = now;
		*s = r;
		return -EIO;
	}
	k_msleep(50);

	uint16_t v;
	bool ok = true;
	ok &= (rd16(REG_VBAT_ADC, &r.vbat_mv) == 0);
	if (rd16(REG_VSYS_ADC, &v) == 0) { r.vsys_mv = v; }
	if (rd16(REG_VBUS_ADC, &v) == 0) { r.vbus_mv = v; }
	if (rd16(REG_IBAT_ADC, &v) == 0) { r.ibat_ma = (int16_t)v; }

	uint8_t st = 0;
	if (i2c_reg_read_byte(i2c, BQ_ADDR, REG_CHG_STAT1, &st) == 0) {
		r.chg_state = (st >> 5) & 0x7;
	}

	r.vbus_present = (r.vbus_mv > 3800);   /* rough: an input source present  */
	r.soc          = soc_from_vbat(r.vbat_mv);
	r.valid        = ok;

	cache = r; cache_ts = now;
	*s = r;
	return ok ? 0 : -EIO;
}

const char *charger_chg_str(uint8_t chg_state)
{
	switch (chg_state) {
	case 0x0: return "idle";      /* not charging          */
	case 0x1: return "trickle";
	case 0x2: return "precharge";
	case 0x3: return "fast";      /* CC                    */
	case 0x4: return "taper";     /* CV                    */
	case 0x6: return "topoff";
	case 0x7: return "done";      /* charge terminated     */
	default:  return "unk";
	}
}
