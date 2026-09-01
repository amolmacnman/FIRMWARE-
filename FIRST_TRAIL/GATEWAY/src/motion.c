/*
 * Debounced motion state machine.
 *
 * A raw motion observation only becomes a CONFIRMED state change after it has
 * persisted continuously for the required time:
 *   stopped -> moving : sustained motion for MOTION_START_CONFIRM_MS
 *   moving -> stopped : sustained stillness for MOTION_STOP_CONFIRM_MS
 * If the raw signal flips back before that time (a 1-min shunt, or a 1-min
 * stop mid-run), the pending transition is cancelled - no event, no cadence
 * flip. Only confirmed transitions call the callback.
 */
#include <zephyr/kernel.h>
#include "motion.h"
#include "app_config.h"

static bool     confirmed;          /* debounced state (false = stopped) */
static bool     pending;            /* a transition is being timed       */
static bool     pending_to;         /* candidate target state            */
static int64_t  since;              /* when the candidate started        */
static void   (*cb)(bool moving);

void motion_init(void (*on_confirmed_change)(bool moving))
{
	cb = on_confirmed_change;
	confirmed = false;
	pending = false;
}

bool motion_is_moving(void) { return confirmed; }
bool motion_active(void)    { return confirmed || pending; }

void motion_sample(bool raw_moving)
{
	int64_t now = k_uptime_get();

	if (raw_moving != confirmed) {
		/* raw disagrees with the confirmed state -> candidate change */
		if (!pending || pending_to != raw_moving) {
			pending = true;
			pending_to = raw_moving;
			since = now;
		} else {
			uint32_t need = raw_moving ? MOTION_START_CONFIRM_MS
						   : MOTION_STOP_CONFIRM_MS;
			if (now - since >= (int64_t)need) {
				confirmed = raw_moving;   /* CONFIRMED */
				pending = false;
				if (cb) {
					cb(confirmed);
				}
			}
		}
	} else {
		/* raw agrees again -> the blip is over, cancel any pending change */
		pending = false;
	}
}
