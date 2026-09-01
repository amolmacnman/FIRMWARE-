/*
 * SHT40 temperature/humidity driver (I2C, addr 0x44 on i2c21).
 * Command 0xFD = measure, high precision. Response = 6 bytes:
 *   T_MSB T_LSB CRC  RH_MSB RH_LSB CRC
 *   T  = -45 + 175 * St/65535   (degC)
 *   RH = -6  + 125 * Srh/65535  (%RH, clamped 0..100)
 */
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include "sht40.h"
#include "power.h"

/*
 * Starting guess. Sensirion sets the address by order code - SHT40-AD1B is
 * 0x44, -BD1B is 0x45, -CD1B is 0x46 - and the schematic records the generic
 * part rather than the variant. sht40_read() therefore latches whichever
 * address answers a CRC-valid measurement and reuses it thereafter, so a
 * substituted part does not silently read as absent in a production build.
 */
#define SHT40_ADDR 0x44

static uint8_t s_addr = SHT40_ADDR;
static const struct device *const i2c = DEVICE_DT_GET(DT_NODELABEL(i2c21));

/* Sensirion CRC-8: poly 0x31, init 0xFF */
static uint8_t crc8(const uint8_t *d, int n)
{
	uint8_t c = 0xFF;
	for (int i = 0; i < n; i++) {
		c ^= d[i];
		for (int b = 0; b < 8; b++) {
			c = (c & 0x80) ? (c << 1) ^ 0x31 : (c << 1);
		}
	}
	return c;
}

int sht40_read(int16_t *temp_x10, uint8_t *rh_pct)
{
	if (!device_is_ready(i2c)) {
		return -1;
	}
	sht_power_on();                 /* SW_SHT_LC on */
	k_msleep(5);                    /* SHT40 boot */

	uint8_t cmd = 0xFD;
	int rc = i2c_write(i2c, &cmd, 1, s_addr);
	if (rc == 0) {
		k_msleep(10);           /* high-precision conversion time */
		uint8_t b[6];
		rc = i2c_read(i2c, b, sizeof(b), s_addr);
		if (rc == 0) {
			if (crc8(&b[0], 2) != b[2] || crc8(&b[3], 2) != b[5]) {
				rc = -EIO;      /* bad CRC */
			} else {
				uint16_t st  = (b[0] << 8) | b[1];
				uint16_t srh = (b[3] << 8) | b[4];
				double T  = -45.0 + 175.0 * st / 65535.0;
				double RH = -6.0 + 125.0 * srh / 65535.0;
				if (RH < 0) RH = 0;
				if (RH > 100) RH = 100;
				*temp_x10 = (int16_t)(T * 10);
				*rh_pct   = (uint8_t)(RH + 0.5);
			}
		}
	}
	/*
	 * On failure, try the other addresses this family ships at - once - and
	 * latch whichever answers. Costs nothing on a healthy board, because the
	 * first attempt succeeds and this is never reached.
	 */
	if (rc != 0) {
		static const uint8_t cand[] = { 0x44, 0x45, 0x46 };

		for (int i = 0; i < (int)ARRAY_SIZE(cand); i++) {
			uint8_t c2 = 0xFD, b2[6];

			if (cand[i] == s_addr) {
				continue;
			}
			if (i2c_write(i2c, &c2, 1, cand[i]) != 0) {
				continue;
			}
			k_msleep(10);
			if (i2c_read(i2c, b2, sizeof(b2), cand[i]) != 0) {
				continue;
			}
			if (crc8(&b2[0], 2) != b2[2] || crc8(&b2[3], 2) != b2[5]) {
				continue;
			}

			s_addr = cand[i];
			printk("SHT40: answering at 0x%02X, not 0x44\n", s_addr);

			uint16_t st  = (b2[0] << 8) | b2[1];
			uint16_t srh = (b2[3] << 8) | b2[4];
			double T  = -45.0 + 175.0 * st / 65535.0;
			double RH = -6.0 + 125.0 * srh / 65535.0;

			if (RH < 0) {
				RH = 0;
			}
			if (RH > 100) {
				RH = 100;
			}
			*temp_x10 = (int16_t)(T * 10);
			*rh_pct   = (uint8_t)(RH + 0.5);
			rc = 0;
			break;
		}
	}

	sht_power_off();                /* SW_SHT_LC off */
	return rc;
}
