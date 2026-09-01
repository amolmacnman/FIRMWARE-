/*
 * Power sequencing for the Smart Wagon gateway (schematic-driven).
 *
 * Load switches (TPS22917) are active-HIGH enable:
 *   SW_LC       -> GNSS main rail        (P0.01)   [+ GNSS_PWR_EN P1.09 conv]
 *   SW_SHT_LC   -> SHT40 rail            (P0.05)
 *   SW_MEM      -> NOR + FRAM rail       (P0.04)
 *   SW_AXI      -> BMA400 rail           (P2.00)   [kept ON = always-on wake]
 *   SW_GNSS     -> shared 3.8V comms rail (P2.08)  [modem VBAT + GNSS front end]
 * GNSS-only control: LC_RESET (P0.02), LC_WKP (P0.03). RESET_N active-LOW.
 * EC200 modem control (sheets 3/4, via NPN transistors -> line pulled LOW):
 *   MODEM_RESET  (P1.13, net GNSS_RESET)  high -> EC200 RESET_N low
 *   MODEM_PWRKEY (P1.14, net GNSS_PWRKEY) high -> EC200 PWRKEY  low (>=500 ms)
 * Charger: NRF_CE (P1.10) — BQ25798 CE is active-LOW (low = charging).
 *
 * VERIFY every polarity against your placed parts before production.
 */
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include "app_config.h"   /* MODEM_PINS_EXPERIMENT - see power_init() */
#include "power.h"
#include "board_pins.h"
#include "watchdog.h"   /* gw_wdt_alive() during the modem boot wait */

/* NOR flash device (rail stays on; we DPD it instead of cutting power). */
static const struct device *const nor = DEVICE_DT_GET(DT_NODELABEL(nor));

/*
 * ---- runtime PM for the two UARTs on switched rails ----------------------
 *
 * uart22 talks to the EC200U, uart30 to the LC29H. Both modules sit on rails
 * this file switches, so the UART is useful for exactly the window between the
 * rail going up and coming down - which makes this the natural place to hold a
 * runtime-PM reference, rather than scattering get/put through the AT parsers
 * where every error path would have to balance them.
 *
 * Holding the reference matters: with CONFIG_PM_DEVICE_RUNTIME an idle UARTE is
 * suspended, and suspending applies the `sleep` pinctrl state, which carries
 * low-power-enable and physically disconnects the pins. Writing to a suspended
 * UARTE therefore succeeds silently and sends nothing.
 *
 * The console (uart20) is NOT managed here - it is not on a switched rail, and
 * suspending it would swallow debug output.
 */
static const struct device *const uart_modem =
	DEVICE_DT_GET(DT_NODELABEL(uart22));
static const struct device *const uart_gnss =
	DEVICE_DT_GET(DT_NODELABEL(uart30));

static bool modem_uart_held;
static bool gnss_uart_held;

/*
 * Take/drop a runtime-PM reference, at most one at a time per device.
 *
 * The `held` flag is not belt-and-braces. link_up() can fail partway through
 * bring-up and main() still calls link_down() afterwards, so the on/off calls
 * are NOT guaranteed to pair up. An unbalanced put would drive the refcount
 * negative and leave the UART suspended for good - the exact failure this
 * whole arrangement exists to prevent - so the flag makes both idempotent.
 */
static void uart_pm_get(const struct device *dev, bool *held)
{
	if (*held || !device_is_ready(dev)) {
		return;
	}
	int rc = pm_device_runtime_get(dev);

	if (rc == 0) {
		*held = true;
	} else {
		printk("PM: runtime_get(%s) failed %d\n", dev->name, rc);
	}
}

static void uart_pm_put(const struct device *dev, bool *held)
{
	if (!*held) {
		return;
	}
	*held = false;
	int rc = pm_device_runtime_put(dev);

	if (rc) {
		printk("PM: runtime_put(%s) failed %d\n", dev->name, rc);
	}
}

/*
 * ---- EARLY RAIL BRING-UP -------------------------------------------------
 *
 * The NOR and FRAM sit on the switched SW_MEM rail, but the spi_nor driver
 * probes its chip during kernel init - long before main() runs. At reset every
 * GPIO is an input, so the TPS22917 enable floats, the rail is DOWN, and the
 * driver's SFDP read returns all zeros:
 *
 *     <err> spi_nor: SFDP magic 00000000 invalid
 *     <err> spi_nor: SFDP read failed: -22
 *
 * The driver then marks the device un-ready for the rest of the boot, so
 * storage_init() later fails with -ENODEV and the whole store-and-forward
 * archive is gone - which is an RDSO s7.5 requirement (one month of data
 * retained through a network outage), not a nice-to-have.
 *
 * Raising the rail inside power_init() cannot fix this: power_init() is called
 * from main(), which is far too late. So the rails that back a DRIVER must come
 * up here instead, at an init priority between the GPIO driver (40) and the
 * spi_nor driver (80).
 *
 * Only the two rails that gate drivers are touched. Everything else stays in
 * power_init() where the sequencing is readable.
 */

/* Read a pin back as an input. Used only to prove a rail write actually took -
 * see the RAILS line in power_init(). */
static int sense(const struct device *port, int pin)
{
	if (!device_is_ready(port)) {
		return -1;
	}
	gpio_pin_configure(port, pin, GPIO_INPUT);
	int v = gpio_pin_get(port, pin);
	gpio_pin_configure(port, pin, GPIO_OUTPUT);
	return v;
}

static int board_rails_early(void)
{
	/*
	 * Report the result rather than discarding it.
	 *
	 * These writes failing silently is indistinguishable from the rails being
	 * up but the devices being absent - and on this board BOTH the NOR and
	 * both I2C parts went quiet at once, which is far more consistent with
	 * the GPIO writes not landing than with three separate dead devices.
	 */
	int r_mem = board_out(PIN_SW_MEM, 1);   /* NOR + FRAM, before spi_nor  */
	int r_axi = board_out(PIN_SW_AXI, 1);   /* BMA400                      */
	/*
	 * VCC_SHT_SC must come up too, and NOT just for the SHT40.
	 *
	 * That rail also carries the I2C PULL-UPS (R4/R5, R8/R9) and the NOR and
	 * FRAM HOLD/WP pull-ups (R11-R15). With it down: every I2C transfer times
	 * out because the bus has no pull-up, and the flash sits with HOLD
	 * asserted so its SFDP read returns zeros.
	 *
	 * Observed on hardware - SW_AXI and SW_MEM both high, yet the BMA400 and
	 * the BQ25798 returned -116 and spi_nor read SFDP as 00000000. One rail,
	 * three devices, two buses.
	 */
	int r_sht = board_out(PIN_SW_SHT_LC, 1);

	printk("RAILS: early SW_MEM=%d SW_AXI=%d SW_SHT=%d%s\n",
	       r_mem, r_axi, r_sht,
	       (r_mem || r_axi || r_sht) ? "   *** GPIO WRITE FAILED ***" : "");

	/* TPS22917 turn-on plus the flash chip's own tPU. Busy-wait rather than
	 * k_msleep: at this init level the scheduler is not yet running threads,
	 * so sleeping here would not reliably yield the delay we need. */
	k_busy_wait(15000);   /* 15 ms */

	return 0;
}

/*
 * POST_KERNEL / 60: after GPIO (40), before SPI_NOR (80). If either of those
 * priorities is ever changed in Kconfig, this number must stay between them.
 */
SYS_INIT(board_rails_early, POST_KERNEL, 60);

/*
 * Power-mux status (TPS2116 ST -> P1.11).
 *
 * The mux switches AUTOMATICALLY in hardware from its own PR-pin threshold;
 * this only observes the result. Reading is safe: ST is an output from the
 * mux, so nothing here can change which rail is live.
 */
static bool s_mux_ok;          /* pin configured and readable */

bool power_mux_is_lto(bool *known)
{
	int v;

	if (!s_mux_ok) {
		if (known) {
			*known = false;
		}
		return true;           /* caller falls back; LTO is the safe guess */
	}
	v = gpio_pin_get(PIN_MUX_ST);
	if (v < 0) {
		if (known) {
			*known = false;
		}
		return true;
	}
	if (known) {
		*known = true;
	}
	return (v == MUX_ST_LTO_LEVEL);
}

static void power_mux_init(void)
{
#if !MUX_ST_READ_ENABLE
	/*
	 * Pin left completely untouched - not configured, not pulled, not read.
	 * This is the exact state of every build that ran without resetting.
	 */
	s_mux_ok = false;
	printk("MUX: ST read DISABLED (MUX_ST_READ_ENABLE=0) - P1.11 untouched; "
	       "src falls back to the SoC estimate\n");
#else
	/* Open-drain status output, so it needs a pull-up to read the idle
	 * (de-asserted) state. */
	s_mux_ok = (board_in(PIN_MUX_ST, GPIO_PULL_UP) == 0);

	if (!s_mux_ok) {
		printk("MUX: ST (P1.11) not readable - src falls back to the SoC "
		       "inference\n");
		return;
	}

	int v = gpio_pin_get(PIN_MUX_ST);
	bool known = false;
	bool lto = power_mux_is_lto(&known);

	/*
	 * Print the RAW level next to the interpretation, so a wrong
	 * MUX_ST_LTO_LEVEL is obvious on the first boot rather than silently
	 * inverting every power report for the life of the product.
	 */
	printk("MUX: ST=%d -> %s selected  (LTO on VIN1; flip MUX_ST_LTO_LEVEL "
	       "if this reads bkp while the board is alive)\n",
	       v, (known && !lto) ? "bkp" : "lto");
#endif /* MUX_ST_READ_ENABLE */
}

void power_init(void)
{
	power_mux_init();

	/*
	 * Opt the two switched-rail UARTs into runtime PM.
	 *
	 * Enabling is explicit rather than relying on the driver's auto-enable:
	 * the nRF UARTE only auto-enables when the devicetree node carries
	 * zephyr,pm-device-runtime-auto, which ours do not, so without this the
	 * devices stay permanently active and the get/put pairs below would be
	 * no-ops that quietly save nothing.
	 *
	 * pm_device_runtime_enable() suspends the device immediately (refcount
	 * starts at zero), which is what we want: nothing has been powered yet
	 * at this point in boot.
	 */
	if (device_is_ready(uart_modem)) {
		pm_device_runtime_enable(uart_modem);
	}
	if (device_is_ready(uart_gnss)) {
		pm_device_runtime_enable(uart_gnss);
	}

	/* BMA400 stays powered so its impact INT can wake the CPU any time, and
	 * so it NEVER needs re-configuration (range/ODR/INT map are retained).
	 * Already raised by board_rails_early(); re-asserted here so this
	 * function still reads as the complete statement of rail policy. */
	board_out(PIN_SW_AXI, 1);

	/* NOR + FRAM rail stays ON. We do NOT gate this rail between reports:
	 * cutting it would desync the flash driver's internal state from a
	 * freshly-powered chip and could corrupt a store-and-forward write in
	 * flight. Instead the flash chip is put in DEEP-POWER-DOWN (~1 uA) by
	 * the storage layer, so the driver stays in sync and needs no re-init. */
	board_out(PIN_SW_MEM, 1);

	/* These rails are cut between reports; each is safe to power-cycle:
	 *   SHT40 is stateless (single-shot measure command, no config to lose);
	 *   GNSS keeps V_BCKP on the always-on rail -> warm start, no re-config. */
	/*
	 * SW_SHT_LC stays ON.
	 *
	 * It was cut here to save power, on the reasoning that the SHT40 is
	 * stateless and safe to power-cycle. That is true of the SHT40 - but the
	 * rail is shared: it also feeds the I2C pull-ups and the NOR/FRAM
	 * HOLD/WP pull-ups. Cutting it silently disabled the I2C bus and held
	 * the flash, so the BMA400, the charger, the NOR and the FRAM all went
	 * quiet together while their own rails read high.
	 *
	 * The standing cost is a few microamps of pull-up leakage plus the
	 * SHT40 idle, which the gateway - mains/solar fed with an LTO pack -
	 * can afford far more easily than losing its store-and-forward archive.
	 */
	board_out(PIN_SW_SHT_LC, 1);
	board_out(PIN_SW_LC, 0);
	board_out(PIN_GNSS_PWR_EN, 0);

	/* Modem off at boot: shared 3.8V comms rail down, PWRKEY/RESET deasserted. */
	board_out(PIN_SW_GNSS, 0);
	board_out(PIN_MODEM_PWRKEY, 0);
	board_out(PIN_MODEM_RESET, 0);

	/* Hold GNSS in reset while unpowered (RESET_N low = reset). */
	board_out(PIN_LC_RESET, 0);
	board_out(PIN_GNSS_RESET, 0);
#if !MODEM_PINS_EXPERIMENT
	board_out(PIN_LC_WKP, 0);
#endif

#if !MODEM_PINS_EXPERIMENT
	/* Keep the charger enabled (CE active-low). */
	board_out(PIN_NRF_CE, 0);
#else
	/*
	 * EXPERIMENT: leave the power-path pins exactly as all_hw_test leaves
	 * them - untouched.
	 *
	 * The early probe proved the modem is silent BEFORE the BLE controller
	 * starts, so the difference is here in power_init or earlier. Comparing
	 * the two programs pin by pin, production drives four that the test never
	 * mentions: LC_WKP (P0.03), NRF_CE (P1.10), MUX_ST (P1.11) and CHG_INT
	 * (P1.15).
	 *
	 * Two of those sit in the power path the modem depends on. The schematic
	 * routes Mux_Out into IC8 (TPS63802), the "Mux To 3.8 V Converter" that
	 * produces 3.8V_GNSS - which U8 gates into GSM_PWR_OP, the EC200U's VBAT.
	 * If MUX_ST's pull-up or NRF_CE's low is steering that mux, the modem
	 * never receives power, which is exactly what a dark NETLIGHT means.
	 *
	 * Leaving them alone costs nothing to try: an unconfigured nRF pin is a
	 * disconnected input, which is the state the working program leaves them
	 * in. If the modem answers with this set, one of these four is the cause
	 * and the next step is to re-enable them one at a time.
	 */
	printk("EXPERIMENT: NRF_CE / MUX_ST / LC_WKP / CHG_INT left untouched\n");
#endif

	/*
	 * Confirm the rails that everything else depends on actually took.
	 *
	 * device_is_ready() on the GPIO port is not enough: a pin can be claimed
	 * by pinctrl for a peripheral, or sit in a domain that is not powered,
	 * and gpio_pin_configure() then fails while the rest of init carries on
	 * as if the rail were up. Reading the pin back is the only proof.
	 */
	printk("RAILS: SW_AXI(P2.00)=%d  SW_MEM(P0.04)=%d  SW_SHT(P0.05)=%d\n",
	       sense(PIN_SW_AXI), sense(PIN_SW_MEM), sense(PIN_SW_SHT_LC));

#if !MODEM_PINS_EXPERIMENT
	/* Status inputs. */
	board_in(PIN_CHG_INT, GPIO_PULL_UP);
	board_in(PIN_MUX_ST, GPIO_PULL_UP);
#endif
}

void gnss_power_on(void)
{
	/* Resume uart30 before the module can talk. The LC29H starts streaming NMEA
	 * on its own the moment reset is released, so the receiver has to be
	 * connected first - a suspended UARTE has its pins detached and would drop
	 * the opening sentences. */
	uart_pm_get(uart_gnss, &gnss_uart_held);

	/* SW_GNSS is the SHARED 3.8V comms rail (GNSS front end + modem VBAT).
	 * acquire_fix() runs before link_up(), so raising it here also readies the
	 * modem's VBAT ahead of modem_power_on(). */
	board_out(PIN_SW_GNSS, 1);
	k_msleep(20);
	board_out(PIN_GNSS_PWR_EN, 1);   /* GNSS 3.3 V LDO */
	k_msleep(10);
	board_out(PIN_SW_LC, 1);         /* GNSS main rail */
	k_msleep(10);
	board_out(PIN_LC_RESET, 1);      /* release reset -> module runs */
	/* Warm start: V_BCKP is on the always-on rail, so ephemeris/time are
	 * preserved and the fix comes quickly. */
}

void gnss_power_off(void)
{
	/* Nothing more will arrive once the rail drops, so let uart30 suspend. */
	uart_pm_put(uart_gnss, &gnss_uart_held);

	board_out(PIN_SW_LC, 0);         /* cut GNSS main rail */
	board_out(PIN_GNSS_PWR_EN, 0);
	board_out(PIN_LC_RESET, 0);      /* hold in reset; keep-alive retained */
	/*
	 * Cut the shared 3.8 V comms rail LAST - it is the outer bracket of the
	 * wake cycle, so the modem (already AT+QPOWD'd in link_down) loses VBAT
	 * here too, giving ~0 draw between reports. RDSO s5.1.20's six-year life
	 * depends on this: the rail is by far the largest load on the board.
	 */
	board_out(PIN_SW_GNSS, 0);
}

void sht_power_on(void)  { board_out(PIN_SW_SHT_LC, 1); k_msleep(3); }
/*
 * Deliberately a NO-OP.
 *
 * VCC_SHT_SC is NOT the SHT40's private rail. It also carries the I2C
 * pull-ups (R4/R5, R8/R9) and the NOR + FRAM HOLD/WP pull-ups (R11-R15), so
 * dropping it disables the whole I2C bus and holds the flash - the BMA400,
 * the BQ25798 and spi_nor all fail together while their own rails read high.
 *
 * That was observed on hardware: -116 from both I2C devices and SFDP reading
 * 00000000. Cutting the rail here would have reintroduced it after the first
 * heartbeat, which is worse than never fixing it - the board would work at
 * boot and then quietly stop.
 *
 * The signature is kept so callers need no change; sht40.c still pairs
 * on/off around a reading and simply gets a rail that stays up.
 */
void sht_power_off(void) { /* see comment - rail is shared, must stay on */ }

/*
 * NOR/FRAM rail is NOT gated (see power_init). "on"/"off" here just take the
 * NOR flash out of / into deep-power-down (~1 uA) via the PM device action,
 * which the has-dpd SPI-NOR driver turns into the RELEASE / ENTER DPD command.
 * FRAM has no DPD (already ~uA idle) and needs nothing. Because the rail is
 * never cut, neither chip ever loses state -> no driver re-init on wake.
 */
/*
 * NOR deep-power-down, only when CONFIG_PM_DEVICE is built in.
 *
 * PM_DEVICE is currently OFF - it was the last configuration difference between
 * this application and the DK build and all_hw_test, both of which talk to the
 * modem happily and enable no power management at all. Losing DPD costs roughly
 * 1 uA of NOR idle; not being able to reach the modem costs everything.
 *
 * Re-enable both together, and re-measure against the s5.1.20 budget.
 */
void mem_power_on(void)
{
#ifdef CONFIG_PM_DEVICE
	if (device_is_ready(nor)) {
		pm_device_action_run(nor, PM_DEVICE_ACTION_RESUME);
	}
#endif
}

void mem_power_off(void)
{
#ifdef CONFIG_PM_DEVICE
	if (device_is_ready(nor)) {
		pm_device_action_run(nor, PM_DEVICE_ACTION_SUSPEND);
	}
#endif
}

void modem_power_on(void)
{
	/* Resume the UART BEFORE the module can send anything. The EC200U emits
	 * its boot banner unprompted once VBAT and PWRKEY are up, and a suspended
	 * UARTE has its pins disconnected, so a late resume loses those bytes. */
	uart_pm_get(uart_modem, &modem_uart_held);

	/* The shared 3.8V rail (SW_GNSS) is normally already up from acquire_fix;
	 * assert it here too so the modem also powers up if called standalone. */
	board_out(PIN_SW_GNSS, 1);

	/*
	 * Power the level shifter BEFORE the modem. Its B-side supply is a GPIO
	 * (see board_pins.h); with it low, nothing the modem says can reach the
	 * nRF and nothing we send can reach the modem. It was never driven, which
	 * makes it the prime suspect for the modem appearing dead on the bench.
	 */
	/*
	 * Raise the WHOLE comms group, not just the modem's own two pins.
	 *
	 * This used to set SW_GNSS, GNSS_3V3 and RESET only, and depend on
	 * gnss_power_on() having already raised GNSS_PWR_EN, SW_LC and LC_RESET
	 * earlier in the cycle. That is a hidden ordering dependency: the modem
	 * cannot be brought up unless the GNSS was powered first, so any path
	 * that skips acquire_fix leaves the modem with a half-built supply.
	 *
	 * GNSS_PWR_EN matters most. It enables IC6 (TPS7A20), and that LDO is
	 * what actually produces GNSS_3V3 - the TXS0102's supply. Driving P2.09
	 * from a GPIO is not a substitute for the regulator being on.
	 *
	 * all_hw_test has all five high when it pulses PWRKEY, because test_gnss()
	 * runs before test_modem(). It brings this modem up on this board every
	 * time; the application, with three of them low, never has. Matching the
	 * sequence that works costs two GPIO writes.
	 */
	/*
	 * Timings copied from all_hw_test's test_gnss(), which brings this
	 * hardware up reliably: 30 ms after the shared rail, 30 ms for the LDO,
	 * 50 ms for the module rail. The application used 10 ms and no delay at
	 * all after SW_GNSS - probably enough, but "probably" is what this whole
	 * investigation has been made of, and matching the working sequence
	 * removes the question.
	 */
	k_msleep(30);
	board_out(PIN_GNSS_PWR_EN, 1);   /* IC6 TPS7A20 -> GNSS_3V3 (TXS0102)  */
	k_msleep(30);
	board_out(PIN_SW_LC, 1);         /* U5 -> GNSS_PWR_OP, IC6's own input */
	k_msleep(50);
	board_out(PIN_LC_RESET, 1);      /* release the GNSS, as the test does */

	board_out(PIN_GNSS_3V3, 1);
	k_msleep(30);                    /* VBAT + translator settle */
	board_out(PIN_MODEM_RESET, 0);   /* deassert reset (EC200 RESET_N = high) */
}

/*
 * PWRKEY is a TOGGLE, not an on switch.
 *
 * Pulsing it at a module that is already running powers it DOWN. This used to
 * be part of modem_power_on() and fired unconditionally, on the reasoning that
 * gnss_power_off() cuts the shared 3.8 V rail at the end of every wake cycle,
 * so the modem has always genuinely lost VBAT by the time we get here.
 *
 * That reasoning holds for a normal cycle and fails for the case that actually
 * matters on a bench: flashing does not power-cycle the board. Run the hardware
 * test, flash the application, and the modem is still up from the previous
 * program - so the first thing the new firmware did was switch it off, and then
 * spend three retries asking a powered-down module to answer AT.
 *
 * It fails the same way after any reset that does not pass through
 * gnss_power_off(): a watchdog bite, a brownout, a debugger reset mid-cycle.
 *
 * So the caller pings first and only pulses when the modem is actually silent.
 * That is idempotent - correct whether the module is off, on, or half-booted -
 * which an unconditional toggle can never be.
 */
void modem_pwrkey_pulse(void)
{
	/*
	 * Report the pin states around the pulse.
	 *
	 * When the modem does not answer, the console gives no way to tell a
	 * module that was never powered from one that was never started from one
	 * whose UART is not connected - and those need three different meters in
	 * three different places. all_hw_test prints this and the application did
	 * not, which is why the same board could pass one and fail the other with
	 * nothing to compare.
	 *
	 * Deliberately NOT read back with sense(): that helper reconfigures the
	 * pin as an input and back, which would momentarily release SW_GNSS -
	 * the modem's own VBAT enable - microseconds before we ask it to start.
	 * Reporting what we drove is worth a line; glitching the supply to
	 * confirm it is not.
	 */
	/*
	 * Let VBAT actually reach the module's minimum before asking it to start.
	 *
	 * Without this the FIRST pulse of every cycle is ignored and a second is
	 * needed - see MODEM_VBAT_SETTLE_MS. That doubles the power-on surges per
	 * cycle, which matters on a supply already browning out at PDP activation.
	 *
	 * Sliced so the watchdog is fed; the report budget is already tight.
	 */
	for (int i = 0; i < MODEM_VBAT_SETTLE_MS / 200; i++) {
		k_msleep(200);
		gw_wdt_alive();
	}

	printk("MODEM: rails up + %d ms VBAT settle - pulsing PWRKEY 600 ms\n",
	       MODEM_VBAT_SETTLE_MS);

	/* Via the NPN, MODEM_PWRKEY high pulls the module's PWRKEY low. */
	board_out(PIN_MODEM_PWRKEY, 1);
	k_msleep(600);                   /* datasheet wants >= 500 ms */
	board_out(PIN_MODEM_PWRKEY, 0);

	/*
	 * WAIT FOR THE MODULE TO BOOT - 20 s, not 3.
	 *
	 * This is the fix for the modem never answering in the application while
	 * all_hw_test and the DK both bring it up on the same hardware.
	 *
	 * The EC200U takes well over ten seconds from PWRKEY to a usable UART.
	 * Three seconds was nowhere near enough, so the AT retries ran against a
	 * module still booting, the cycle gave up, and the NEXT cycle pulsed
	 * PWRKEY again - which is a TOGGLE, so it switched off the module that
	 * had finally come up. That is why the NETLIGHT only started blinking
	 * after the LAST pulse of a run: every earlier pulse was being undone by
	 * the next one.
	 *
	 * all_hw_test never hit this because its modem was usually already
	 * running from a previous pass, so its "ping first" check succeeded and
	 * it never pulsed at all.
	 */
	/*
	 * Sleep in 2 s slices, feeding the watchdog between them.
	 *
	 * A single k_msleep(20000) here was long enough - together with the
	 * 40 s GNSS timeout and the AT retries after it - to push one report
	 * cycle past the 120 s watchdog and RESET THE BOARD. That reset runs
	 * power_init(), which drops SW_GNSS and therefore the modem's VBAT, so
	 * the module was power-cycled before it had ever finished booting. Every
	 * cycle started from cold, which is exactly why the application could
	 * never reach the state all_hw_test gets to on its second run.
	 *
	 * The console showed it plainly: "RAILS: SW_GNSS held up" followed
	 * immediately by the MCUboot banner. all_hw_test has no watchdog at all,
	 * so its 5 s and 8 s waits cost it nothing.
	 */
	for (int i = 0; i < 10; i++) {
		k_msleep(2000);
		gw_wdt_alive();
	}
	printk("MODEM: PWRKEY released, 20 s boot settle done\n");
	/* Modem boots; UART is ready within a few seconds. ec200_modem_up() retries
	 * AT until it responds, so no fixed post-delay is needed here. */
}

void modem_power_off(void)
{
	/* The caller (link_down) has already issued AT+QPOWD for a graceful
	 * shutdown. The modem's VBAT is the SHARED 3.8V rail (SW_GNSS), which also
	 * feeds the GNSS front end, so it is cut by release_fix()/gnss_power_off()
	 * at the end of the wake cycle - NOT here. Nothing else to gate. */

	/* Drop the UART reference so the UARTE can suspend. Safe here: the caller
	 * has already sent AT+QPOWD and will not talk to the modem again this
	 * cycle. */
	uart_pm_put(uart_modem, &modem_uart_held);
}
