#ifndef MOTION_H
#define MOTION_H
#include <stdbool.h>

/*
 * Debounced motion state. A raw motion sample only becomes a CONFIRMED state
 * change after it persists for the configured time (MOTION_START_CONFIRM_MS /
 * MOTION_STOP_CONFIRM_MS). Brief blips (e.g. a 1-min shunt, or a 1-min stop
 * mid-run) are filtered out - no event, no cadence flip.
 */
void motion_init(void (*on_confirmed_change)(bool moving));

/* Feed one raw motion observation (true = motion sensed now). Call from the
 * BMA400 (always-on) and, when available, from the GNSS speed at a fix. */
void motion_sample(bool raw_moving);

/* Confirmed (debounced) state - use THIS for cadence and TRAIN_START/STOP. */
bool motion_is_moving(void);

/* True while moving OR a transition is pending - i.e. keep sampling. When this
 * is false the train is firmly stopped and polling can stop (rely on the
 * BMA400 any-motion interrupt to wake on the next start). */
bool motion_active(void);

#endif /* MOTION_H */
