/*
 * Gateway watchdog (nRF54L15 internal WDT31 -> full SoC reset).
 *
 * The gateway has NO external watchdog (unlike the sub-nodes, which have a
 * TPL5010), yet it is the single point of failure for the whole wagon: it owns
 * the modem, the MQTT session, the BLE scan and the OTA path. If it wedges, the
 * wagon goes silent until someone physically reaches it. This module uses the
 * SoC's own watchdog so no board change is needed.
 *
 * WHY NOT A PLAIN "FEED IT IN THE MAIN LOOP" WATCHDOG:
 * the main loop blocks on k_event_wait(K_FOREVER) and a stopped wagon reports
 * only every INTERVAL_IDLE_MS (12 h by default). Feeding solely from the loop
 * would demand a >12 h timeout, which detects almost nothing. Instead a periodic
 * timer feeds the hardware, but only while the system is demonstrably healthy:
 *
 *   - blocked in the event wait AND still inside its expected sleep budget, or
 *   - actively running and making progress (the loop bumps a counter)
 *
 * Anything else - spinning without progress, a deadlock, or an idle that
 * overruns its budget because a wake-up was lost - stops the feed and the WDT
 * resets the SoC. That last case matters: a lost timer would otherwise leave the
 * gateway asleep forever, looking perfectly healthy.
 *
 * NOTE: the nRF watchdog cannot be reconfigured or stopped once started, and it
 * keeps counting in System ON idle. That is what we want here.
 */
#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <stdint.h>
#include <stdbool.h>

/* Install and start WDT31. Safe to call once, at boot. 0 on success; on failure
 * the other calls become no-ops so the gateway still runs unprotected rather
 * than not at all. */
int  gw_wdt_init(void);

/* Called from the main loop each iteration: "I am running and progressing". */
void gw_wdt_alive(void);

/* Bracket the long idle sleep. `budget_ms` is the interval the scheduler just
 * armed; the watchdog tolerates that plus GW_WDT_IDLE_MARGIN_MS before it
 * treats the sleep as a lost wake-up. */
void gw_wdt_idle_begin(uint32_t budget_ms);
void gw_wdt_idle_end(void);

#endif /* WATCHDOG_H */
