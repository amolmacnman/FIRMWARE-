/*
 * Per-wagon sensor-node roster (VARIABLE length).
 * The number of nodes is NOT fixed - a wagon may have 4, 5, 10, up to 20.
 * Edit nodes.c to list exactly the nodes fitted to THIS wagon; everything
 * else (heartbeat sens[], missing-node reporting, provisioning) sizes itself
 * from this table automatically.
 */
#ifndef NODES_H
#define NODES_H

#include <stdint.h>
#include "sensor_proto.h"

struct wagon_node {
	uint8_t     id;    /* node_id as advertised (unique within the wagon) */
	uint8_t     type;  /* enum sw_node_type                               */
	const char *pos;   /* PHYSICAL location label for the server/reader,
			    * e.g. "B1-A1-L" = bogie 1, axle 1, left. Free text;
			    * set it to your fitment convention. "" if n/a.     */
};

extern const struct wagon_node WAGON_NODES[];
extern const int               WAGON_NODE_COUNT;   /* set from the table */

/*
 * FITMENT MASK - one bit per node id. Bit N set = node id N is fitted.
 *
 *   0x7FFFF  all 19 nodes
 *   0x20000  tank-temp only (id 17)
 *   0x000FF  the eight bearings, nothing else
 *
 * This is the address a group OTA is aimed at: wagons sharing a mask are
 * fitted alike and can therefore share one firmware image.
 *
 * Derived from WAGON_NODES, never declared by hand. A number maintained
 * separately in app_config.h would be a second thing to update when editing
 * nodes.c, and the day someone forgets is the day a fleet update goes to
 * wagons that cannot run it - undetectably, because a stale number still looks
 * valid. Here the roster IS the number.
 *
 * A mask rather than a hash so it can be READ. 0x7FFFF says "everything";
 * 0x7FFEF says "everything except id 4". A CRC says 27833 and 51902 - equally
 * correct for detecting a mismatch, useless for understanding one, and
 * impossible to check by eye before a release goes out.
 *
 * IDS ONLY, not types. That is safe because id implies type by project
 * convention (sw_std_type_for_id below), and boot verifies the roster obeys
 * it. Encoding a type per node would need 57 bits and buy nothing while the
 * convention holds.
 */
uint32_t wagon_fitment_id(void);        /* EFFECTIVE mask (FRAM or image) */
uint32_t wagon_fitment_image(void);     /* what this image was built with */

/*
 * The effective roster, derived from the effective mask.
 *
 * Use these instead of WAGON_NODES / WAGON_NODE_COUNT anywhere the answer
 * should follow a runtime override. WAGON_NODES remains the IMAGE default and
 * the source of custom position labels; a node added by downlink that is not
 * in the table gets its label from the standard convention.
 */
int wagon_node_count(void);
const struct wagon_node *wagon_node_at(int i);   /* NULL if i out of range */

/* Standard position label for an id, e.g. "B1-A1-L". "" outside the map. */
const char *sw_std_pos_for_id(uint8_t id);

/*
 * The standard id -> type map for a 19-node wagon. Returns 0 for an id outside
 * the convention.
 *
 * This is what makes an id-only mask sufficient: every wagon in the fleet
 * agrees that id 12 is the handbrake and ids 13..16 are doors, so listing WHICH
 * ids are present fully describes the fitment. nodes_check_convention() warns
 * at boot if a roster departs from it, because the mask would then be
 * ambiguous between two wagons.
 */
uint8_t sw_std_type_for_id(uint8_t id);

/* Warn (loudly, at boot) if WAGON_NODES departs from the standard map. */
void nodes_check_convention(void);

#endif /* NODES_H */
