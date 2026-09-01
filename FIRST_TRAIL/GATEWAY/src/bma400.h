#ifndef BMA400_H
#define BMA400_H
#include <stdbool.h>

/* Configure BMA400 (I2C 0x14): ±16 g wide range so a real impact is not
 * clipped, and a high-g/activity interrupt routed to INT1 (P1.16) for
 * impact/tamper wake. Returns 0 on success (chip id verified). */
int bma400_init(void);

/* Read acceleration in g (x,y,z). */
int bma400_read_g(double *x, double *y, double *z);

/* Vector magnitude in g (for the impact alarm 'val'). */
double bma400_magnitude_g(void);

/* Is the wagon moving right now? Bursts a few samples and looks at the
 * variance of |a| - vibration => moving. Cheap; the debounce upstream makes
 * the decision robust. */
bool bma400_is_moving(void);

#endif
