#ifndef SHT40_H
#define SHT40_H
#include <stdint.h>
/* Read SHT40 climate. Powers the SHT rail on/off around the read.
 * Returns 0 on success; fills temperature x10 (degC) and humidity (%RH). */
int sht40_read(int16_t *temp_x10, uint8_t *rh_pct);
#endif
