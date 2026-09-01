#ifndef POWER_H
#define POWER_H

/*
 * Rail / power sequencing using the schematic control GPIOs (board_pins.h).
 * TPS22917 load switches are active-HIGH enable. The BMA400 rail (SW_AXI) is
 * powered at init and kept on (always-on impact wake); everything else is off
 * until needed to hold the sleep budget.
 */
void power_init(void);

/*
 * Which rail the power mux has selected, read from the TPS2116 ST pin.
 *
 * Returns true for LTO (VIN1), false for the Li-SOCl2 backup (VIN2). *known is
 * set false when the pin could not be read, in which case the return value is
 * a safe default and the caller should fall back to its own estimate.
 *
 * This is what Rev.1 s2.2 means by "read from the power-mux status, not
 * inferred". The mux switches automatically in hardware; nothing in firmware
 * commands it.
 */
bool power_mux_is_lto(bool *known);

void gnss_power_on(void);    /* main GNSS rail on, reset released (warm start) */
void gnss_power_off(void);   /* main rail off; V_BCKP keep-alive stays (HW)    */

void sht_power_on(void);     /* SHT40 rail                                     */
void sht_power_off(void);

void mem_power_on(void);     /* NOR + FRAM rail (store-and-forward)             */
void mem_power_off(void);

void modem_power_on(void);   /* rails + level shifter + reset deasserted        */

/*
 * Pulse PWRKEY. Call ONLY after AT has been tried and got no answer - PWRKEY
 * is a toggle, so pulsing a running module powers it down. See power.c.
 */
void modem_pwrkey_pulse(void);
void modem_power_off(void);  /* gate modem rail (after a graceful AT+QPOWD)     */

#endif /* POWER_H */
