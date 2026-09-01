/*
 * BMA400 accelerometer driver (I2C, addr 0x14 - SDO tied low per schematic).
 * Wide range (±16 g) so a high-energy impact is captured at true magnitude;
 * a high-g (generic) interrupt is routed to INT1 -> P1.16 for impact/tamper
 * wake. The rail (SW_AXI) is kept powered by power_init() (always-on wake).
 *
 * NOTE: interrupt thresholds are first-cut; tune GEN1 threshold/duration to
 * your shock profile and RDSO impact/derailment criteria.
 */
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <math.h>
#include "bma400.h"
#include "app_config.h"   /* MOTION_ACC_VAR, IMPACT_G_THRESH */

#define BMA_ADDR        0x14
#define REG_CHIPID      0x00   /* = 0x90 */
#define REG_ACC_X_LSB   0x04
#define REG_ACC_CONFIG0 0x19   /* power mode */
#define REG_ACC_CONFIG1 0x1A   /* range + ODR */
#define REG_INT_CONFIG0 0x1F
#define REG_INT1_MAP    0x21
#define REG_GEN1CONFIG0 0x3F   /* generic interrupt 1 config block */

static const struct device *const i2c = DEVICE_DT_GET(DT_NODELABEL(i2c21));

static int wr(uint8_t reg, uint8_t val)
{
	uint8_t b[2] = { reg, val };
	return i2c_write(i2c, b, 2, BMA_ADDR);
}
static int rd(uint8_t reg, uint8_t *buf, int n)
{
	return i2c_write_read(i2c, BMA_ADDR, &reg, 1, buf, n);
}

int bma400_init(void)
{
	if (!device_is_ready(i2c)) {
		return -1;
	}
	/*
	 * Distinguish a FAILED BUS from a WRONG CHIP.
	 *
	 * This used to print "chip id 0x%02x" using an id left at 0 when the read
	 * errored, so a dead I2C bus reported itself as "chip id 0x00" - which
	 * reads as a real part answering with a wrong value. The two need
	 * completely different investigations: a bus error points at the rail, the
	 * pull-ups, or an unpopulated part; a genuine mismatch means the wrong
	 * device is fitted. The errno separates them at a glance.
	 */
	uint8_t id = 0;
	int rc = rd(REG_CHIPID, &id, 1);

	if (rc != 0) {
		printk("BMA400: I2C read failed (%d) - check the SW_AXI rail "
		       "(P2.00), the bus pull-ups, and that the part is "
		       "populated\n", rc);
		return -ENODEV;
	}
	if (id != 0x90) {
		printk("BMA400: wrong chip id 0x%02x (expected 0x90)\n", id);
		return -ENODEV;
	}

	/* ACC_CONFIG1: range ±16g (bits7:6=3), ODR 100 Hz (0x08) -> 0xC8 */
	wr(REG_ACC_CONFIG1, 0xC8);
	/* ACC_CONFIG0: normal power mode (0x02). For lowest power use 0x01
	 * (low-power) once the interrupt path is tuned. */
	wr(REG_ACC_CONFIG0, 0x02);
	k_msleep(5);

	/* Route generic interrupt 1 (high-g/activity) to INT1 (-> P1.16).
	 * GEN1 threshold/duration left at reset here; set per shock profile. */
	wr(REG_INT1_MAP, 0x04);        /* map GEN1 -> INT1 (verify bit) */
	wr(REG_INT_CONFIG0, 0x04);     /* enable GEN1 (verify bit)      */

	printk("BMA400: ready (±16g, INT1->P1.16)\n");
	return 0;
}

int bma400_read_g(double *x, double *y, double *z)
{
	uint8_t b[6];
	if (rd(REG_ACC_X_LSB, b, 6) != 0) {
		return -EIO;
	}
	/* Right-aligned 12-bit two's complement: LSB = bits[7:0],
	 * MSB low nibble = bits[11:8]. Sign-extend from bit 11. */
	int16_t rx = (int16_t)(((b[1] & 0x0F) << 8) | b[0]);
	int16_t ry = (int16_t)(((b[3] & 0x0F) << 8) | b[2]);
	int16_t rz = (int16_t)(((b[5] & 0x0F) << 8) | b[4]);
	if (rx & 0x0800) rx -= 4096;
	if (ry & 0x0800) ry -= 4096;
	if (rz & 0x0800) rz -= 4096;
	/* ±16g full scale, 12-bit signed -> g = raw * 16 / 2048 */
	const double s = 16.0 / 2048.0;
	*x = rx * s; *y = ry * s; *z = rz * s;
	return 0;
}

double bma400_magnitude_g(void)
{
	double x, y, z;
	if (bma400_read_g(&x, &y, &z) != 0) {
		return 0.0;
	}
	return sqrt(x * x + y * y + z * z);
}

bool bma400_is_moving(void)
{
	/* Burst-sample |a| and use its variance. A still wagon reads ~1 g with
	 * tiny variance; a rolling wagon vibrates -> higher variance. */
	double sum = 0.0, sum2 = 0.0;
	int n = 0;
	for (int i = 0; i < 16; i++) {
		double x, y, z;
		if (bma400_read_g(&x, &y, &z) == 0) {
			double m = sqrt(x * x + y * y + z * z);
			sum += m;
			sum2 += m * m;
			n++;
		}
		k_msleep(5);
	}
	if (n < 4) {
		return false;
	}
	double mean = sum / n;
	double var  = (sum2 / n) - (mean * mean);
	return var > MOTION_ACC_VAR;   /* g^2 threshold (tune on hardware) */
}
