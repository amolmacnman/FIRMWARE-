/*
 * board_pins.h — control GPIOs of the gateway, taken directly from
 * main_node_schematic_nrf54l15. Include this where the firmware needs to drive
 * power sequencing (GNSS rail around a fix, memory rail for store-and-forward,
 * etc.). Comms pins (GNSS/EC200 UART, I2C, BMA_INT) are in app.overlay.
 *
 * Port map: P0.x -> gpio0, P1.x -> gpio1, P2.x -> gpio2.
 */
#ifndef BOARD_PINS_H
#define BOARD_PINS_H

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#define GPIO0 DEVICE_DT_GET(DT_NODELABEL(gpio0))
#define GPIO1 DEVICE_DT_GET(DT_NODELABEL(gpio1))
#define GPIO2 DEVICE_DT_GET(DT_NODELABEL(gpio2))

/* ---- GNSS (LC29H) control ---- */
#define PIN_GNSS_PWR_EN   GPIO1, 9    /* enable GNSS 3.3V converter        */
#define PIN_SW_LC         GPIO0, 1    /* switched GNSS main rail (on@fix)  */
#define PIN_LC_RESET      GPIO0, 2    /* LC29H reset                       */
#define PIN_LC_WKP        GPIO0, 3    /* keep-alive / wakeup (warm start)  */
#define PIN_GNSS_RESET    GPIO1, 13
#define PIN_GNSS_PWRKEY   GPIO1, 14

/* ---- sensor / memory power switches ---- */
#define PIN_SW_AXI        GPIO2, 0    /* BMA400 accelerometer rail         */
#define PIN_SW_SHT_LC     GPIO0, 5    /* SHT40 rail                        */
#define PIN_SW_MEM        GPIO0, 4    /* NOR + FRAM rail (store-and-forward)*/

/* ---- power path / charger ---- */
#define PIN_POWER_SWITCH  GPIO1, 12
#define PIN_MUX_ST        GPIO1, 11   /* power-mux status/select           */
#define PIN_CHG_QON       GPIO1, 8    /* charger QON                       */
#define PIN_CHG_INT       GPIO1, 15   /* charger interrupt (input)         */
#define PIN_NRF_CE        GPIO1, 10   /* BQ25798 charger CE (active-LOW)   */

/* ---- EC200U modem (schematic sheets 3/4) ----
 * SW_GNSS gates the shared 3.8V comms rail (U8 TPS22917 -> 3.8V buck IC8 ->
 * net 3.8V_GNSS), which powers BOTH the EC200 modem VBAT AND the GNSS front
 * end. Active-HIGH. This is the modem's power enable.
 *
 * PWRKEY and RESET_N reach the EC200 through NPN transistors, so the nRF pin is
 * ACTIVE-HIGH but pulls the modem line LOW:
 *   MODEM_PWRKEY high -> T2 on -> EC200 PWRKEY low   (turn-on pulse, >=500 ms)
 *   MODEM_RESET  high -> T1 on -> EC200 RESET_N low  (reset asserted)
 * (The nets are labelled GNSS_* on the combined board but wire to the EC200.)
 * VERIFY polarity/timing on the bench before production. */
#define PIN_SW_GNSS       GPIO2, 8    /* shared 3.8V comms rail enable      */

/*
 * TXS0102 level-shifter supply, net GNSS_3V3 -> P2.09.
 *
 * This GPIO is not a control line, it is the SUPPLY (VCCB) for the translator
 * sitting between the nRF UART (P1.06/P1.07) and the EC200U's MAIN_TXD/MAIN_RXD.
 * The netlist shows the net reaching exactly two pins: U1.21 here and B2.7 on
 * the shifter. Nothing else drives it.
 *
 * While it is low the translator is unpowered and that UART is dead in BOTH
 * directions - the modem can be perfectly healthy and still appear silent.
 * The A side is fed from the EC200U's own VDD_EXT, so B must be raised for the
 * link to work at all.
 *
 * A GPIO as a rail is unusual, so METER B2 pin 7 before trusting this.
 */
#define PIN_GNSS_3V3      GPIO2, 9
#define PIN_MODEM_PWRKEY  GPIO1, 14   /* net GNSS_PWRKEY -> EC200 PWRKEY (T2)*/
#define PIN_MODEM_RESET   GPIO1, 13   /* net GNSS_RESET  -> EC200 RESET_N(T1)*/

/* ---- accelerometer interrupt (also exposed as the bma-int alias) ---- */
#define PIN_BMA_INT       GPIO1, 16

/* ---- convenience helpers ---- */
/*
 * GPIO_OUTPUT | GPIO_INPUT, not GPIO_OUTPUT alone.
 *
 * Connecting the input buffer as well costs nothing and makes board_level()
 * below possible: the pad can then be READ while still being driven. Without
 * it there is no way to tell "the firmware wrote 1" from "the pin is at 3.3 V",
 * and those differ whenever something external holds a line down.
 *
 * That distinction cost a bring-up session. The modem would not answer, every
 * rail had been driven high, and there was no way to check any of it without a
 * meter on a board already covered in probes.
 */
/*
 * PLAIN GPIO_OUTPUT - exactly what all_hw_test's drive() does.
 *
 * This briefly used GPIO_OUTPUT | GPIO_INPUT so board_level() could read a pin
 * back while it was still driving. That was added to diagnose the modem, and
 * the NETLIGHT - which had been blinking after a PWRKEY pulse - stopped
 * entirely from that build onwards.
 *
 * PWRKEY reaches the module through T2, a BC847B whose base sees roughly
 * 0.55 mA through R58 (4.7k). That is well inside a standard-drive pin, but it
 * is the one signal on this board that must actually sink current rather than
 * just present a level, and it is the only pin whose behaviour changed. The
 * readback was a debugging convenience; starting the modem is not.
 *
 * So: match the helper that demonstrably works, and get readback some other way
 * if it is ever needed again.
 */
static inline int board_out(const struct device *port, int pin, int val)
{
	int r = gpio_pin_configure(port, pin, GPIO_OUTPUT);

	if (r == 0) {
		gpio_pin_set(port, pin, val);
	}
	return r;
}

/*
 * Read a driven pin back AT THE PAD. Returns 0/1, or -1 if it cannot be read.
 *
 * Non-destructive: the pin keeps driving. Note this detects a pin held down
 * externally - a short, or a load switch pulling against us - but NOT an open
 * circuit: an unsoldered pin reads back the nRF's own drive quite happily.
 */
static inline int board_level(const struct device *port, int pin)
{
	if (!device_is_ready(port)) {
		return -1;
	}
	return gpio_pin_get(port, pin);
}

static inline int board_in(const struct device *port, int pin, gpio_flags_t extra)
{
	return gpio_pin_configure(port, pin, GPIO_INPUT | extra);
}

/*
 * Usage:
 *   board_out(PIN_SW_LC, 1);     // power the GNSS rail on before a fix
 *   board_out(PIN_SW_LC, 0);     // power it off after
 *   board_out(PIN_SW_MEM, 1);    // power memory before store-and-forward
 *   board_in (PIN_CHG_INT, GPIO_PULL_UP);
 */

#endif /* BOARD_PINS_H */
