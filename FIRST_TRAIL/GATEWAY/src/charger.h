#ifndef CHARGER_H
#define CHARGER_H
#include <stdint.h>
#include <stdbool.h>

/*
 * TI BQ25798 (BQ25798RQMR) buck-boost charger with a built-in 16-bit ADC.
 * The gateway's LTO pack is rechargeable, so - unlike the primary-cell
 * subnodes - we read the real pack voltage / current / charge state from the
 * charger over I2C (bus i2c21, addr 0x6B) rather than the nRF SAADC.
 */
struct charger_status {
	uint16_t vbat_mv;      /* battery pack voltage        (1 mV/LSB)         */
	uint16_t vsys_mv;      /* system rail voltage                            */
	uint16_t vbus_mv;      /* input (solar/USB) voltage                      */
	int16_t  ibat_ma;      /* battery current: + charging, - discharging     */
	uint8_t  chg_state;    /* raw CHG_STAT field (REG1C[7:5])                */
	bool     vbus_present; /* an input source is supplying power             */
	uint8_t  soc;          /* 0..100, mapped from vbat (LTO curve)           */
	bool     valid;        /* false if the last I2C read failed              */
};

int         charger_init(void);                        /* probe the BQ25798   */
int         charger_read(struct charger_status *s);    /* cached (<=1/20 s)   */
const char *charger_chg_str(uint8_t chg_state);        /* "fast","done",...   */

#endif /* CHARGER_H */
