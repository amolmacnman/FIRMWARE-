/*
 * Gateway watchdog - see watchdog.h for the rationale behind the liveness gate.
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/sys/atomic.h>

#include "watchdog.h"

/* Hardware timeout. Comfortably longer than the feed period so a single missed
 * tick (e.g. a burst of BLE interrupts) never causes a spurious reset, but short
 * enough that a genuinely wedged gateway comes back in ~2 minutes. */
#define GW_WDT_TIMEOUT_MS       120000u

/* How often the liveness gate runs and (if healthy) feeds the hardware. */
#define GW_WDT_FEED_MS          30000u

/* Slack added to the scheduler's armed interval before an idle is judged to be
 * a lost wake-up. Generous: timer drift and a late BLE/motion event are normal,
 * a wake-up that never arrives is not. */
#define GW_WDT_IDLE_MARGIN_MS   (10u * 60u * 1000u)

static const struct device *const wdt = DEVICE_DT_GET(DT_ALIAS(watchdog0));

static int      s_channel = -1;
static bool     s_ready;

static atomic_t s_progress;        /* bumped by gw_wdt_alive()               */
static atomic_t s_idle;            /* 1 while blocked in the event wait      */
static int64_t  s_idle_start;      /* uptime when the idle began             */
static uint32_t s_idle_budget_ms;  /* interval the scheduler armed           */

static void feed_fn(struct k_timer *t);
K_TIMER_DEFINE(gw_wdt_timer, feed_fn, NULL);

/*
 * Runs in timer (ISR) context. Feeding is a single register write, so it is
 * safe here - and deliberately here rather than in a thread, so that a wedged
 * scheduler stops the feed and triggers the reset.
 */
static void feed_fn(struct k_timer *t)
{
	ARG_UNUSED(t);

	static atomic_val_t last_progress;
	bool healthy;

	if (atomic_get(&s_idle)) {
		/* Sleeping is normal - but only for as long as the schedule says.
		 * Overrunning means a wake-up was lost and nobody is coming. */
		int64_t slept = k_uptime_get() - s_idle_start;

		healthy = slept < (int64_t)s_idle_budget_ms + GW_WDT_IDLE_MARGIN_MS;
	} else {
		/* Awake: only healthy if the main loop actually moved since the
		 * previous check. A spin or deadlock freezes this counter. */
		atomic_val_t now = atomic_get(&s_progress);

		healthy = (now != last_progress);
		last_progress = now;
	}

	if (healthy) {
		wdt_feed(wdt, s_channel);
	}
}

int gw_wdt_init(void)
{
	if (!device_is_ready(wdt)) {
		printk("WDT: device not ready - gateway runs UNPROTECTED\n");
		return -ENODEV;
	}

	struct wdt_timeout_cfg cfg = {
		.window.min = 0,
		.window.max = GW_WDT_TIMEOUT_MS,
		.callback   = NULL,             /* no pre-warning: reset directly */
		.flags      = WDT_FLAG_RESET_SOC,
	};

	s_channel = wdt_install_timeout(wdt, &cfg);
	if (s_channel < 0) {
		printk("WDT: install failed (%d) - gateway runs UNPROTECTED\n",
		       s_channel);
		return s_channel;
	}

	/* PAUSE_HALTED_BY_DBG so sitting on a breakpoint does not reset the
	 * board mid-debug. Has no effect in the field. */
	int rc = wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
	if (rc != 0) {
		printk("WDT: setup failed (%d) - gateway runs UNPROTECTED\n", rc);
		return rc;
	}

	s_ready = true;
	atomic_set(&s_idle, 0);
	wdt_feed(wdt, s_channel);
	k_timer_start(&gw_wdt_timer, K_MSEC(GW_WDT_FEED_MS), K_MSEC(GW_WDT_FEED_MS));

	printk("WDT: armed, %u ms timeout\n", GW_WDT_TIMEOUT_MS);
	return 0;
}

void gw_wdt_alive(void)
{
	atomic_inc(&s_progress);
}

void gw_wdt_idle_begin(uint32_t budget_ms)
{
	if (!s_ready) {
		return;
	}
	s_idle_budget_ms = budget_ms;
	s_idle_start     = k_uptime_get();
	atomic_set(&s_idle, 1);
}

void gw_wdt_idle_end(void)
{
	if (!s_ready) {
		return;
	}
	atomic_set(&s_idle, 0);
	atomic_inc(&s_progress);
}
