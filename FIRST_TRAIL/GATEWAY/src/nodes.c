/*
 * ============================================================
 *  PER-WAGON ROSTER - edit ONLY this list for each wagon.
 * ============================================================
 * List the sensor nodes actually fitted to this wagon. The count is derived
 * automatically, so 4, 5, 10 or 20 nodes all "just work". node_id must match
 * what each node advertises (NODE_ID in that node's app_config.h) and be
 * unique within the wagon.
 *
 * The 3rd field is the PHYSICAL POSITION label, sent to the server so the
 * reader knows exactly which bogie/axle/wheel a reading belongs to - important
 * once a fleet has many wagons, each with 2 bogies x 2 axles x 2 sides. Use
 * your own convention; here "B<bogie>-A<axle>-<L|R>" for bearings.
 *
 * Example below: a 2-bogie wagon with 8 bearings (2 axles x 2 sides x 2 bogies),
 * 4 load/tilt (front + rear on each bogie), 1 handbrake, 4 doors (left + right
 * on each bogie), 2 temperature (one per bogie) = 19 nodes. Edit to match the
 * real fitment.
 */
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include "nodes.h"
#include "sw_ids.h"
#include "config.h"
#include <stdbool.h>

const struct wagon_node WAGON_NODES[] = {
	// { 0,  SW_TYPE_BEARING,   "B1-A1-L" },
	// { 1,  SW_TYPE_BEARING,   "B1-A1-R" },
	// { 2,  SW_TYPE_BEARING,   "B1-A2-L" },
	// { 3,  SW_TYPE_BEARING,   "B1-A2-R" },
	// { 4,  SW_TYPE_BEARING,   "B2-A1-L" },
	// { 5,  SW_TYPE_BEARING,   "B2-A1-R" },
	// { 6,  SW_TYPE_BEARING,   "B2-A2-L" },
	// { 7,  SW_TYPE_BEARING,   "B2-A2-R" },
	// { 8,  SW_TYPE_LOAD_TILT, "B1LT-L" },   /* bogie 1, load+tilt, left  */
	// { 9,  SW_TYPE_LOAD_TILT, "B1LT-R" },   /* bogie 1, load+tilt, right */
	// { 10, SW_TYPE_LOAD_TILT, "B2LT-L" },   /* bogie 2, load+tilt, left  */
	// { 11, SW_TYPE_LOAD_TILT, "B2LT-R" },   /* bogie 2, load+tilt, right */
	// { 12, SW_TYPE_HANDBRAKE, "HB"       },
	// { 13, SW_TYPE_DOOR,      "B1D-L"    },   /* bogie 1, door, left  */
	// { 14, SW_TYPE_DOOR,      "B1D-R"    },   /* bogie 1, door, right */
	// { 15, SW_TYPE_DOOR,      "B2D-L"    },   /* bogie 2, door, left  */
	// { 16, SW_TYPE_DOOR,      "B2D-R"    },   /* bogie 2, door, right */
	{ 17, SW_TYPE_TANK_TEMP, "B1T"      },   /* bogie 1 temperature */
	{ 18, SW_TYPE_TANK_TEMP, "B2T"      },   /* bogie 2 temperature */
};

const int WAGON_NODE_COUNT = ARRAY_SIZE(WAGON_NODES);

uint8_t sw_std_type_for_id(uint8_t id)
{
	if (id <= 7)  { return SW_TYPE_BEARING;    }   /* 2 bogies x 2 axles x 2 sides */
	if (id <= 11) { return SW_TYPE_LOAD_TILT;  }   /* front + rear, each bogie     */
	if (id == 12) { return SW_TYPE_HANDBRAKE;  }
	if (id <= 16) { return SW_TYPE_DOOR;       }   /* left + right, each bogie     */
	if (id <= 18) { return SW_TYPE_TANK_TEMP;  }   /* one per bogie                */
	return 0;
}

uint32_t wagon_fitment_image(void)
{
	uint32_t m = 0;

	for (int i = 0; i < WAGON_NODE_COUNT; i++) {
		uint8_t id = WAGON_NODES[i].id;

		if (id < 32) {
			m |= (uint32_t)1u << id;
		}
	}
	return m;
}

uint32_t wagon_fitment_id(void)
{
	uint32_t o = cfg_fitment();

	return o ? o : wagon_fitment_image();
}

const char *sw_std_pos_for_id(uint8_t id)
{
	static const char *const p[19] = {
		"B1-A1-L", "B1-A1-R", "B1-A2-L", "B1-A2-R",
		"B2-A1-L", "B2-A1-R", "B2-A2-L", "B2-A2-R",
		"B1LT-L",  "B1LT-R",  "B2LT-L",  "B2LT-R",
		"HB",
		"B1D-L",   "B1D-R",   "B2D-L",   "B2D-R",
		"B1T",     "B2T",
	};

	return id < 19 ? p[id] : "";
}

/*
 * The effective roster, rebuilt whenever the mask changes.
 *
 * Cached rather than recomputed per call because gwalarm and the heartbeat walk
 * it on every cycle, and it only changes when a downlink alters the override.
 */
static struct wagon_node s_eff[19];
static int      s_eff_n = -1;
static uint32_t s_eff_mask;

static void eff_rebuild(void)
{
	uint32_t m = wagon_fitment_id();

	if (s_eff_n >= 0 && m == s_eff_mask) {
		return;
	}
	s_eff_mask = m;
	s_eff_n    = 0;

	for (uint8_t id = 0; id < 19; id++) {
		if (!(m & ((uint32_t)1u << id))) {
			continue;
		}
		s_eff[s_eff_n].id   = id;
		s_eff[s_eff_n].type = sw_std_type_for_id(id);
		s_eff[s_eff_n].pos  = sw_std_pos_for_id(id);

		/* Prefer a label from the table - the fitter's own convention
		 * beats ours for anything actually listed there. */
		for (int k = 0; k < WAGON_NODE_COUNT; k++) {
			if (WAGON_NODES[k].id == id && WAGON_NODES[k].pos &&
			    WAGON_NODES[k].pos[0]) {
				s_eff[s_eff_n].pos = WAGON_NODES[k].pos;
				break;
			}
		}
		s_eff_n++;
	}
}

int wagon_node_count(void)
{
	eff_rebuild();
	return s_eff_n;
}

const struct wagon_node *wagon_node_at(int i)
{
	eff_rebuild();
	return (i >= 0 && i < s_eff_n) ? &s_eff[i] : NULL;
}

void nodes_check_convention(void)
{
	/*
	 * The mask records ids, not types, so it only identifies a fitment
	 * while every wagon agrees what each id means. If this wagon calls id 5
	 * a door and the rest of the fleet calls it a bearing, two genuinely
	 * different fitments produce the SAME mask - and a group OTA would send
	 * one of them the wrong image with nothing to catch it.
	 *
	 * A warning, not a refusal: a deliberate non-standard fitment is a
	 * legitimate thing to build, and bricking it at boot would be a worse
	 * failure than the one being guarded against. But it must not pass in
	 * silence, so name the offending node and say what it breaks.
	 */
	for (int i = 0; i < WAGON_NODE_COUNT; i++) {
		uint8_t id  = WAGON_NODES[i].id;
		uint8_t std = sw_std_type_for_id(id);

		if (std && WAGON_NODES[i].type != std) {
			printk("nodes: WARNING id %u is type %u, convention says "
			       "%u - fitment mask 0x%05X is AMBIGUOUS, do not "
			       "use it for a group OTA\n",
			       id, WAGON_NODES[i].type, std,
			       wagon_fitment_image());
		}
	}
}
