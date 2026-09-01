#ifndef THERMAL_H
#define THERMAL_H
#include <stdbool.h>

/*
 * Thermal transmit halt - RDSO WD-35-MISC-2024 s5.1.10:
 *
 *   "The device should feature an internal temperature sensor with the ability
 *    to halt transmissions ensuring safety and longevity."
 *
 * Reads the nRF54L15 on-die sensor and gates the cellular modem, which is by
 * far the largest heat source on the board. Records produced while halted are
 * still buffered to the NOR ring, so nothing is lost - only the transmission
 * is deferred until the gateway has cooled.
 */
int  thermal_init(void);

/* True while transmission must be suppressed. */
bool thermal_tx_blocked(void);

/* Latest die temperature in degC x10, or THERMAL_NA if unreadable. */
#define THERMAL_NA  (-32768)
int16_t thermal_die_temp_x10(void);

/* Re-evaluate the halt state. Call once per main-loop pass. */
void thermal_poll(void);

#endif /* THERMAL_H */
