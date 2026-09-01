/*
 * Thermal transmit halt (RDSO WD-35-MISC-2024 s5.1.10).
 *
 * WHY THIS EXISTS: the spec requires an internal temperature sensor able to
 * halt transmissions "ensuring safety and longevity". The EC200U in a transmit
 * burst is the dominant heat source in the enclosure; suppressing it is the
 * only lever the firmware actually has.
 *
 * THRESHOLD CHOICE: the spec sets no number - s7.15 explicitly makes threshold
 * values the vendor's responsibility, to be agreed with Indian Railways within
 * six months. The value here is therefore derived, not copied:
 *
 *   s5.1.9 requires reliable operation from -20 C to +85 C. +85 C is the top of
 *   the guaranteed envelope, so transmitting above it is operating outside what
 *   the device is specified to do. THERMAL_HALT_C is set there.
 *
 * HYSTERESIS: a wide 10 C release band. A gateway that halts at 85 C and
 * resumes at 84 C would oscillate, and each resume attempt is itself a heat
 * pulse - the failure mode would be a device that never cools. Resuming at
 * 75 C forces a genuine recovery before the modem is allowed back.
 *
 * WHAT IS *NOT* HALTED: data capture. Records keep being produced and buffered
 * to the NOR ring exactly as they are when the network is down, so a thermal
 * event costs latency, never data (s7.5 requires a month of storage anyway).
 *
 * MEASUREMENT CAVEAT: this is the SoC die, not the modem or ambient. The die
 * sits near the modem on this board and tracks enclosure temperature with some
 * lag, so it is a proxy. It reads high during a transmit burst and low between
 * bursts, which is the correct bias for a protective cutout: the peak is what
 * matters.
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#include "thermal.h"
#include "app_config.h"

static const struct device *const die = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(temp));

static bool    s_blocked;
static int16_t s_last_x10 = THERMAL_NA;

int thermal_init(void)
{
	if (die == NULL || !device_is_ready(die)) {
		/*
		 * Fail OPEN, not closed. A dead temperature sensor must not
		 * silence a wagon's telemetry - losing all reporting is a worse
		 * outcome than running without the protective cutout, and s7.14
		 * wants sensor faults reported rather than acted on blindly.
		 */
		printk("THERM: die sensor unavailable - halt protection OFF\n");
		return -ENODEV;
	}
	printk("THERM: ready (halt >= %d C, resume <= %d C)\n",
	       THERMAL_HALT_C, THERMAL_RESUME_C);
	return 0;
}

int16_t thermal_die_temp_x10(void)
{
	return s_last_x10;
}

bool thermal_tx_blocked(void)
{
	return s_blocked;
}

void thermal_poll(void)
{
	struct sensor_value v;

	if (die == NULL || !device_is_ready(die)) {
		return;
	}
	if (sensor_sample_fetch(die) != 0 ||
	    sensor_channel_get(die, SENSOR_CHAN_DIE_TEMP, &v) != 0) {
		s_last_x10 = THERMAL_NA;
		return;                  /* keep the previous halt state */
	}

	int32_t c_x10 = v.val1 * 10 + v.val2 / 100000;

	s_last_x10 = (int16_t)c_x10;

	if (!s_blocked && c_x10 >= THERMAL_HALT_C * 10) {
		s_blocked = true;
		printk("THERM: %d.%d C - TX HALTED (s5.1.10), buffering\n",
		       c_x10 / 10, (c_x10 < 0 ? -c_x10 : c_x10) % 10);
	} else if (s_blocked && c_x10 <= THERMAL_RESUME_C * 10) {
		s_blocked = false;
		printk("THERM: %d.%d C - TX resumed\n",
		       c_x10 / 10, (c_x10 < 0 ? -c_x10 : c_x10) % 10);
	}
}
