/*
 * Shared BLE advertising payload for Smart Wagon sensor nodes.
 *
 * MULTI-WAGON ISOLATION: every node advertises its wagon-group id (wgn_group).
 * A gateway is configured with its own wgn_group and DISCARDS any advert whose
 * group does not match - so neighbouring wagons' nodes (within BLE range in a
 * yard) never contaminate this wagon's data. node_id is unique only within a
 * wagon; the gateway maps (wgn_group,node_id) -> node serial for the protocol.
 *
 * Keep this file identical in the gateway and sub-node projects.
 */
#ifndef SENSOR_PROTO_H
#define SENSOR_PROTO_H

#include <stdint.h>
#include <stdbool.h>

#define SW_COMPANY_ID   0xFFFF   /* test company ID; use your BT SIG ID in prod */
#define SW_PROTO_VER    2        /* 2 = ENCRYPTED advert (AES-CCM); 1 = legacy */

enum sw_node_type {
	SW_TYPE_BEARING   = 1,   /* btemp + vibration */
	SW_TYPE_LOAD_TILT = 2,   /* load + tilt       */
	SW_TYPE_HANDBRAKE = 3,   /* applied/released  */
	SW_TYPE_TANK_TEMP = 4,   /* cargo temperature */
	SW_TYPE_DOOR      = 5,   /* open/closed       */
	SW_TYPE_BRAKE     = 6,   /* air-brake pressures */
};

#define SW_FLAG_ALARM     (1u << 0)
#define SW_FLAG_LOWBATT   (1u << 1)
#define SW_FLAG_DOOR_OPEN (1u << 2)  /* door node: reed reports open      */
#define SW_FLAG_IMPACT    (1u << 3)  /* accel: impact / tamper detected    */
#define SW_FLAG_TILT      (1u << 4)  /* accel: tilt beyond threshold       */
/*
 * Bearing node: vibration index is above the node's own vib threshold.
 *
 * This is a HINT, NOT an alarm. The FLAT_WHEEL decision stays with the
 * gateway, because it is only valid above VIBRATION_MIN_KMH and the bearing
 * node has no accelerometer and no speed input - it cannot tell rolling from
 * shunting, so letting it alarm autonomously would fire during yard handling.
 *
 * What the flag buys is LATENCY. The node escalates to FAST_PERIOD_MS while it
 * is set, so the gateway gets fresh vibration data every ~1 s instead of every
 * 30 s. Before this, a flat wheel could sit unreported for a whole advert
 * period while an over-temperature on the same node was reported within a
 * second - the node escalated for its OWN alarm but not for a gateway-decided
 * one. RDSO s7.3 asks for a "near real time alert" once a threshold is
 * exceeded; this closes the gap without moving the speed gate off the gateway.
 */
#define SW_FLAG_VIB_HIGH  (1u << 5)  /* bearing: vibration over node limit  */

/*
 * "No reading" sentinel for the value/value2 fields.
 *
 * A sensor that fails to answer must not be reported as a plausible number:
 * a silent STS4x reading 0.0 C looks exactly like a cold wagon. INT16_MIN is
 * -3276.8 C once scaled, outside any physical range, so it can never collide
 * with real data.
 *
 * Producers (sub-nodes) seed value/value2 with this before every read.
 * Consumers (gateway) MUST test for it and publish JSON null rather than the
 * raw number - see telemetry.c.
 */
#define SW_VAL_NA  ((int16_t)-32768)

/*
 * struct sw_adv is now the INTERNAL (plaintext) representation used on-chip
 * after decryption - it is no longer what goes on air. The on-air format is
 * struct sw_adv_enc (below). The gateway decrypts sw_adv_enc into an sw_adv so
 * the rest of the pipeline is unchanged.
 */
struct sw_adv {
	uint16_t company_id;   /* = SW_COMPANY_ID              */
	uint8_t  proto_ver;    /* = SW_PROTO_VER               */
	uint16_t wgn_group;    /* wagon group id (isolation)   */
	uint8_t  node_type;    /* enum sw_node_type            */
	uint8_t  node_id;      /* unique within the wagon 0..19 */
	uint8_t  seq;          /* low byte of the nonce counter */
	uint8_t  flags;        /* SW_FLAG_*                    */
	uint8_t  batt;         /* node battery %               */
	int16_t  value;        /* primary reading              */
	int16_t  value2;       /* secondary reading            */
} __attribute__((packed));

/*
 * MEANING OF value / value2 - the contract every node and the gateway share.
 *
 * The payload has exactly two int16 slots, so a node cannot report its primary
 * measurement, its internal temperature AND a shock magnitude at once. The
 * rule that resolves it:
 *
 *   value   Always the node's PRIMARY measurement, in that type's own units:
 *             TANK_TEMP  RTD cargo temperature  x10 degC
 *             DOOR       STS4X internal temp    x10 degC (state is in flags)
 *             BEARING    bearing temperature    x10 degC
 *
 *   value2  |a| x100 (centi-g) WHENEVER SW_FLAG_IMPACT is set, on every node
 *           type. Otherwise type-specific:
 *             TANK_TEMP  STS4X internal temperature x10 degC
 *             DOOR       |a| x100 centi-g (this node always reports it)
 *             others     reserved
 *
 * The flag is what disambiguates, so no extra byte is needed. This used to be
 * undefined, and the gateway simultaneously assumed value2 was a temperature
 * (heartbeat "t") and a shock magnitude (impact alarm "cg") - each correct for
 * one node type and wrong for the other.
 *
 * Either slot may be SW_VAL_NA when its sensor did not answer.
 */
/* 6-byte plaintext reading - THIS is the part that gets encrypted. */
struct sw_adv_pt {
	uint8_t  flags;        /* SW_FLAG_*      */
	uint8_t  batt;         /* battery %      */
	int16_t  value;        /* primary        */
	int16_t  value2;       /* secondary      */
} __attribute__((packed));

/*
 * 21-byte ENCRYPTED on-air advert (fits one legacy BLE manufacturer-data AD):
 *   [ 11-byte cleartext header - also the AES-CCM AAD (authenticated) ]
 *   [ 6-byte ciphertext of sw_adv_pt ] [ 4-byte MIC (auth tag) ]
 * The header stays clear so the gateway can filter by wgn_group and rebuild the
 * decryption nonce; it is authenticated, so it cannot be tampered with.
 */
struct sw_adv_enc {
	uint16_t company_id;   /* = SW_COMPANY_ID                       */
	uint8_t  proto_ver;    /* = SW_PROTO_VER (2)                    */
	uint16_t wgn_group;    /* isolation group (from wagon number)   */
	uint8_t  node_type;    /* enum sw_node_type                     */
	uint8_t  node_id;      /* unique within the wagon               */
	uint32_t ctr;          /* monotonic nonce counter (per node)    */
	uint8_t  ct[6];        /* AES-CCM ciphertext of sw_adv_pt       */
	uint8_t  mic[4];       /* AES-CCM authentication tag            */
} __attribute__((packed));

/* protocol typ string for the heartbeat sens[] element */
static inline const char *sw_typ_str(uint8_t t)
{
	switch (t) {
	case SW_TYPE_BEARING:   return "btemp";
	case SW_TYPE_LOAD_TILT: return "load";
	case SW_TYPE_HANDBRAKE: return "hbrake";
	case SW_TYPE_TANK_TEMP: return "ttemp";
	case SW_TYPE_DOOR:      return "door";
	case SW_TYPE_BRAKE:     return "brake";
	default:                return "unk";
	}
}

/* alarm code (protocol §3.2) for a node type in alarm */
static inline const char *sw_alarm_code(uint8_t t, int moving)
{
	switch (t) {
	case SW_TYPE_BEARING:   return "HOT_BEARING";
	case SW_TYPE_LOAD_TILT: return "TILT";
	case SW_TYPE_HANDBRAKE: return moving ? "HANDBRAKE_MOVING" : "TAMPER";
	case SW_TYPE_TANK_TEMP: return "TANK_OVERTEMP";
	case SW_TYPE_DOOR:      return "DOOR_UNAUTH";
	default:                return "SENSOR_FAULT";
	}
}

/* =========================================================================
 *  DOWNLINK (gateway -> sub-node), carried over GATT during the node's
 *  connectable window. Keep this block identical in gateway and sub-node.
 * =========================================================================
 *
 * The uplink advert is authenticated with the per-wagon key; a config write
 * MUST be too, or anyone in BLE range during the ~4 s window could rewrite a
 * wagon's alarm thresholds. So the downlink reuses the SAME AES-CCM primitive
 * and the SAME per-wagon key, with two additions:
 *
 *   1. DIRECTION SEPARATION. Byte 7 of the nonce is 0x00 for uplink and 0x01
 *      for downlink. Without this an uplink and a downlink could land on the
 *      same nonce with the same key, which breaks CCM catastrophically (it
 *      leaks the keystream). The uplink wire format is unchanged - byte 7 was
 *      already a zero pad - so this is backward compatible.
 *   2. A SEPARATE COUNTER. The node persists the highest downlink ctr it has
 *      accepted and rejects anything <= that, so a captured write cannot be
 *      replayed later.
 */

/* Downlink command ids (sw_dn_pt.cmd). */
#define SW_DN_SET_THRESHOLD  0x01   /* set alarm threshold + hysteresis */

/*
 * Set the BMA400 tamper/impact threshold, in MILLI-g, in sw_dn_pt.threshold.
 * hyst is unused and must be 0.
 *
 * Only nodes that actually carry a BMA400 (the Tank_and_Door schematic
 * variants - door and tank) implement this. A node built without bma400.c
 * falls through downlink.c's switch to the default case and rejects the frame
 * with VALUE_NOT_ALLOWED, WITHOUT advancing its replay counter - so an
 * all-nodes broadcast is safe: sensor-less nodes simply refuse it.
 *
 * Milli-g rather than g because the frame carries an int16 and the gateway
 * already stores its own impact limit as impact_mg. 2000 = 2.0 g.
 */
#define SW_DN_SET_IMPACT     0x02   /* set BMA400 tamper threshold (milli-g) */

/*
 * Set the bearing node's VIBRATION hint threshold, as a raw index, in
 * sw_dn_pt.threshold. hyst is unused and must be 0.
 *
 * This threshold does NOT decide the FLAT_WHEEL alarm - the gateway still
 * does that, gated on speed. It only decides when the node raises
 * SW_FLAG_VIB_HIGH and escalates to FAST_PERIOD_MS, so keep it at or below
 * the gateway's FLAT_WHEEL_VIB_THRESH: a node limit ABOVE the gateway's would
 * leave a band where the gateway would alarm but the node never escalates,
 * putting the latency straight back.
 *
 * Only bearing nodes implement this; every other type rejects it with
 * VALUE_NOT_ALLOWED and does not advance its replay counter.
 */
#define SW_DN_SET_VIB        0x03   /* set bearing vibration hint threshold */

/*
 * Enter a temporary FAST CONFIG WINDOW. sw_dn_pt.threshold carries how long to
 * stay in fast mode, in SECONDS (1..3600); hyst is unused and must be 0.
 *
 * Normally a node is connectable for CFGWIN_MS every CFGWIN_PERIOD_MS (4 s in
 * 600 s), which costs ~53 uA of the ~190 uA six-year budget and means a queued
 * command waits ~5 minutes on average. That is the right trade for a fit-and-
 * forget node, but it is painful while commissioning a rake, and far too slow
 * to carry a firmware image.
 *
 * This makes the node open its window every CFGWIN_FAST_PERIOD_MS instead, for
 * the requested number of seconds, then revert AUTOMATICALLY. It reverts on
 * timeout even if the gateway never comes back, so a node can never be left
 * burning battery because a campaign was abandoned.
 *
 * Cost is bounded and tiny: the higher duty applies only while the timer runs.
 * Ten minutes of fast mode is well under 1 mAh of a 10,000 mAh cell, and the
 * standing budget is completely unchanged - a node that is never put into fast
 * mode draws exactly what it drew before.
 */
#define SW_DN_FASTWIN        0x04   /* temporary fast config window (seconds) */

/*
 * ---- sub-node OTA control (the bulk data itself goes on the img char) ----
 *
 * BEGIN carries the image size in sw_dn_pt.threshold/hyst as a 32-bit value
 * split low/high, because the command frame has no room for anything wider.
 * It erases the secondary slot and arms the receiver.
 *
 * END asks the node to verify what it received and, if the image is sound,
 * mark it for MCUboot to swap on the next reset. threshold carries the
 * expected CRC16 of the whole image so a truncated or reordered transfer is
 * refused BEFORE it is ever booted; MCUboot's own signature check is the
 * second, independent gate.
 *
 * ABORT discards a partial transfer and re-arms the node. It exists so a failed
 * campaign does not leave the receiver holding state until the next reset.
 */
#define SW_DN_OTA_BEGIN      0x05   /* erase slot, arm receiver, size in lo/hi */
#define SW_DN_OTA_END        0x06   /* verify CRC + request MCUboot upgrade    */
#define SW_DN_OTA_ABORT      0x07   /* discard a partial transfer              */

/*
 * Set the advertising cadence, in SECONDS.
 *
 *   threshold -> quiet period   (no alarm active)
 *   hyst      -> alarm period   (SW_FLAG_ALARM / IMPACT / DOOR_OPEN set)
 *
 * Reuses the existing 6-byte payload rather than growing it: both values are
 * small enough for the int16 fields, so the CCM buffers stay the same size.
 *
 * The node clamps to sane limits before applying - see downlink.c. A cadence
 * this mechanism could set too low would flatten the cell in weeks, and a
 * node too slow to report is useless, so neither end is left to the caller.
 */
#define SW_DN_SET_CADENCE    0x08

/*
 * SW_DN_SET_NODEID - provision this node's id. "value" is the new id.
 *
 * Addressed to the node's CURRENT id, so it is a re-provisioning tool, not a
 * way to reach a node whose id is unknown. A virgin node still gets its first
 * id from the build seed at manufacture; after that the NVS copy survives
 * every OTA, which is what lets one image serve a whole node type.
 *
 * Takes effect from the next advert rather than on reboot: this arrives during
 * a config window, and restarting would drop the link before the gateway saw
 * the response.
 */
#define SW_DN_SET_NODEID     0x09

/*
 * SW_DN_SET_BATT - the cell's two VOLTAGE references, in millivolts.
 *
 *   threshold -> "fresh" mV : what a healthy, recently-fitted cell reads
 *   hyst      -> "eol"   mV : the end-of-life knee
 *
 * THESE DO NOT DEFINE A STATE-OF-CHARGE MAP AND MUST NOT BE USED AS ONE.
 * Li-SOCl2 holds ~3.6 V for ~95 % of its life and then falls off a cliff, so a
 * voltage->percent curve reads ~100 % for years and then 0 within weeks - worse
 * than no gauge, because it looks trustworthy. The node's SoC stays
 * coulomb-counted in batt.c. These two values feed the parts of the gauge that
 * voltage genuinely can answer:
 *
 *   fresh - gate on the cell-replacement detector. A power-on reset only counts
 *           as "somebody fitted a new cell" if the cell ALSO reads at least
 *           this, so a cell dying under brownout - which power-cycles the node
 *           too - can never reset the accumulator to 100 %.
 *   eol   - independent end-of-life backstop. The coulomb counter drifts over a
 *           six-year life; the voltage knee does not, so crossing it pins the
 *           reported percentage down regardless of what the counter believes.
 *
 * Settable over the air because both are chemistry constants that are ESTIMATED
 * at build time and only become known once a cell batch has been characterised
 * at temperature. The alternative to a downlink is opening a sealed node and
 * reflashing it over SWD.
 */
#define SW_DN_SET_BATT       0x0A

/*
 * SW_DN_SET_BATTCAL - calibrate the coulomb-counting fuel gauge.
 *
 *   rsvd      -> WHICH constant (SW_BATTCAL_*)
 *   threshold -> the value, in that constant's own units
 *   hyst      -> unused, send 0
 *
 * This is the one command that makes the gauge trustworthy. A sub-node's
 * state-of-charge is not measured, it is INTEGRATED: every wake adds
 * (current x time) for each phase, so the gauge is only ever as accurate as the
 * per-phase currents in app_config.h - all of which ship as estimates marked
 * MEASURE:, because none can be known before a PPK2 has been put in series with
 * a real cell on real hardware.
 *
 * rsvd carries the selector so all seven constants share one opcode. It has
 * been a reserved zero since the protocol was written, so using it keeps the
 * payload at six bytes and reuses the existing CCM buffers unchanged.
 *
 * IMPORTANT - THIS DOES NOT RE-INTEGRATE THE PAST. The accumulator holds charge
 * already counted using the OLD constants; a correction applies from the next
 * wake onward and cannot undo error that has already accrued. Calibrate before
 * deployment where possible, and treat this as a way to stop error growing for
 * the REMAINING life rather than a way to repair a year of it.
 */
#define SW_DN_SET_BATTCAL    0x0B

/* Selector values for SW_DN_SET_BATTCAL's rsvd byte. Order is part of the wire
 * protocol - append only, never renumber. */
#define SW_BATTCAL_I_SLEEP     0   /* uA  - System ON idle, BLE idle          */
#define SW_BATTCAL_I_SELFDISCH 1   /* uA  - the cell's own leakage            */
#define SW_BATTCAL_I_MEAS      2   /* uA  - sensor rail on + conversion       */
#define SW_BATTCAL_T_MEAS      3   /* ms  - how long that phase lasts         */
#define SW_BATTCAL_I_TX        4   /* uA  - mean over a non-connectable burst */
#define SW_BATTCAL_I_CFGWIN    5   /* uA  - connectable advertising window    */
#define SW_BATTCAL_USABLE_MAH  6   /* mAh - derated usable cell capacity      */
#define SW_BATTCAL_COUNT       7

/*
 * SW_DN_SET_TXPWR - BLE output power in dBm. "threshold" carries the value,
 * SIGNED, so negative levels work; "hyst" is unused.
 *
 * Settable over the air because the right level is a property of the
 * INSTALLATION, not of the firmware: how far the node sits from its gateway,
 * how much steel is in between, and how much of the six-year budget that
 * particular wagon can spare. One image cannot know any of that.
 *
 * THIS SPENDS BATTERY, AND STEEPLY. dBm is logarithmic but PA current is not,
 * so the cost is far from proportional to the number. Measured against the
 * 190 uA six-year ceiling at the 30 s cadence:
 *
 *     0 dBm   134 uA   8.5 yr    the shipped default
 *    +4 dBm   ~1.5x radio, close to the line
 *    +8 dBm   327 uA   3.5 yr    FAILS - the PA is least efficient near maximum
 *
 * The node applies whatever it is told inside the range below and prints the
 * level the controller ACTUALLY selected - the hardware silently clamps values
 * it cannot produce, so a request for more than the part can deliver looks
 * identical to one that worked.
 */
#define SW_DN_SET_TXPWR      0x0C

/*
 * NO SW_DN_SET_WAGON, deliberately.
 *
 * sw_dn_pt carries SIX bytes - cmd, rsvd and two 16-bit operands. An 11-digit
 * wagon number does not fit, and splitting it across frames would need chunk
 * indices, reassembly state and a commit step: a small protocol of its own, to
 * set a value exactly once in a node's life.
 *
 * It is also not needed. The goal of moving the wagon number into NVS was to
 * make it SURVIVE an update, so one image can serve every node of a type on
 * every wagon - not to make it settable over the air. Nodes are already
 * flashed individually when they are fitted, so the build seed provisions NVS
 * on first boot and every later OTA leaves it alone.
 *
 * Re-provisioning a node onto a DIFFERENT wagon therefore means a reflash over
 * SWD. That is the right cost: it happens when a node is physically moved
 * between wagons, which is already a workshop operation.
 */

/*
 * Highest addressable node id + 1.
 *
 * A protocol constant, not a gateway one: node_id travels in the advert header
 * and both sides must agree on its range. It lived in the gateway's
 * ble_sensors.h, which left the sub-nodes unable to bounds-check an id a
 * downlink asked them to adopt - guarded here so that older header can keep
 * its identical definition without colliding.
 */
#ifndef SW_MAX_NODES
#define SW_MAX_NODES 20
#endif

/*
 * Advertising cadence bounds, in seconds.
 *
 * Protocol limits rather than node policy, so both ends can enforce them: the
 * floor is the alarm cadence itself (anything faster buys nothing and costs
 * battery), and the ceiling is set by the gateway's 15-minute staleness check -
 * a node heard less often than that is flagged DOWN and raises SENSOR_FAULT
 * continuously.
 *
 * These lived only in each node's app_config.h, which left the gateway unable
 * to reject an impossible cadence before sealing it - so a bad value came back
 * as "queued" and failed silently at the node minutes later.
 */
#ifndef CADENCE_MIN_S
#define CADENCE_MIN_S    1u
#endif
#ifndef CADENCE_MAX_S
#define CADENCE_MAX_S    3600u
#endif

/*
 * Is this threshold physically reachable on this node type?
 *
 * Bounds are the SENSOR's range, not a policy about sensible operating points.
 * Anything inside is accepted even if unusual; anything outside can never fire
 * and is therefore a mistake rather than a decision.
 *
 *   bearing      -40.0 .. 150.0 C   front-end range
 *   tank/cargo   -50.0 .. 150.0 C   RDSO 5.4.2
 *   load/tilt        0 .. 100 %     the node reports a percent
 *   door, brake      0 or 1         binary comparison
 *
 * An unknown type returns true: a node type added later is not this function's
 * to judge, and silently refusing its thresholds would be worse than not
 * checking them.
 */
static inline bool sw_threshold_sane(uint8_t node_type, int16_t thr, int16_t hyst)
{
	int16_t lo, hi;

	switch (node_type) {
	case SW_TYPE_BEARING:   lo = -400; hi = 1500; break;
	case SW_TYPE_TANK_TEMP: lo = -500; hi = 1500; break;
	case SW_TYPE_LOAD_TILT: lo = 0;    hi = 100;  break;
	case SW_TYPE_HANDBRAKE:
	case SW_TYPE_DOOR:      lo = 0;    hi = 1;    break;
	default:                return true;
	}

	if (thr < lo || thr > hi) {
		return false;
	}
	if (hyst < 0) {
		return false;          /* negative hysteresis inverts the release */
	}
	if (hi - lo <= 1) {
		return hyst == 0;      /* binary state has no hysteresis band */
	}
	/*
	 * The RELEASE point must be reachable too. thr - hyst below the sensor
	 * floor gives an alarm that can set but never clear - which looks like a
	 * stuck sensor and hides the real reading behind a latched alarm.
	 */
	return (int32_t)thr - (int32_t)hyst >= (int32_t)lo;
}

/*
 * BMA400 impact limit, milli-g. Matches the 0.5 .. 16 g the gateway clamps its
 * own accelerometer to - the part cannot resolve outside +/-16 g, and 0 would
 * latch an impact alarm permanently on the first vibration.
 */
static inline bool sw_impact_sane(int16_t milli_g)
{
	return milli_g >= 500 && milli_g <= 16000;
}

/*
 * Li-SOCl2 voltage references, millivolts (SW_DN_SET_BATT).
 *
 * The bounds are the cell's physically possible range, not a preference. An
 * ER34615 sits at ~3.6 V nominal, reads ~3.67 V open-circuit fresh, and is long
 * dead below 2 V, so anything outside 2000..4000 mV is a typo or a unit mix-up
 * - volts sent where millivolts were meant, which is the easy mistake here
 * because both are small integers.
 *
 * Rejecting those matters more than it looks: this command can only ever
 * WEAKEN the gauge if it is wrong. A `fresh` above the cell's real voltage
 * permanently disarms the replacement detector, so a genuinely new cell reports
 * the old one's depletion forever. An `eol` below the cell's floor silently
 * removes the end-of-life backstop, leaving nothing to catch counter drift.
 * Neither failure announces itself - the node keeps advertising happily.
 *
 * The 100 mV separation stops the two crossing or coinciding, which would make
 * one cell simultaneously "fresh enough to be new" and "past end of life".
 */
static inline bool sw_batt_mv_sane(int16_t fresh_mv, int16_t eol_mv)
{
	if (fresh_mv < 2000 || fresh_mv > 4000) {
		return false;
	}
	if (eol_mv < 2000 || eol_mv > 4000) {
		return false;
	}
	return (int32_t)fresh_mv - (int32_t)eol_mv >= 100;
}

/*
 * Range-check one gauge calibration value (SW_DN_SET_BATTCAL).
 *
 * The ceilings come from the budget the gauge exists to track: 13 Ah over a
 * six-year life is 247 uA AVERAGE, total, across every phase. A sleep current
 * of 10 mA or a self-discharge of 1 mA is therefore not a tuning choice, it is
 * a typo or a unit mix-up (mA sent where uA was meant), and accepting it would
 * drain the modelled cell to 0 % in days while the real one sits full.
 *
 * Zero is allowed for the currents - a phase genuinely measured as negligible
 * should be settable to nothing - but NOT for the two values used as divisors
 * or durations: a zero capacity divides by zero in batt_read_pct(), and a zero
 * measurement time silently stops that phase being counted at all.
 */
static inline bool sw_battcal_sane(uint8_t sel, int16_t v)
{
	if (v < 0) {
		return false;          /* every one of these is a magnitude */
	}
	switch (sel) {
	case SW_BATTCAL_I_SLEEP:     return v <= 10000;
	case SW_BATTCAL_I_SELFDISCH: return v <= 1000;
	case SW_BATTCAL_I_MEAS:      return v <= 30000;
	case SW_BATTCAL_T_MEAS:      return v >= 1 && v <= 10000;
	case SW_BATTCAL_I_TX:        return v <= 30000;
	case SW_BATTCAL_I_CFGWIN:    return v <= 30000;
	case SW_BATTCAL_USABLE_MAH:  return v >= 100 && v <= 30000;
	default:                     return false;   /* unknown selector */
	}
}

/*
 * BLE output power, dBm (SW_DN_SET_TXPWR).
 *
 * -40 is the lowest level any nRF5 part offers and +8 is the nRF54L15 ceiling,
 * so this is the hardware's range rather than a policy. Out-of-range values are
 * refused instead of clamped: the controller would quietly clamp them itself,
 * and an operator who asked for +20 deserves to be told it is impossible rather
 * than left believing the link was strengthened.
 *
 * The BUDGET limit is lower than the hardware limit - about +4 dBm at the 30 s
 * cadence - but that is a function of cadence and cell size, both of which are
 * themselves settable, so it is not enforced here. The node logs the cost
 * instead. See SW_DN_SET_TXPWR above.
 */
static inline bool sw_txpwr_sane(int16_t dbm)
{
	return dbm >= -40 && dbm <= 8;
}

/* Plaintext downlink payload - 6 bytes, same size as sw_adv_pt so it reuses
 * the identical CCM buffers. */
struct sw_dn_pt {
	uint8_t  cmd;          /* SW_DN_*                                 */
	uint8_t  rsvd;         /* 0, reserved for future command operands */
	int16_t  threshold;    /* new alarm threshold (node units)        */
	int16_t  hyst;         /* new hysteresis                          */
} __attribute__((packed));

/* On-air downlink frame, written to the node's config characteristic. Same
 * layout as sw_adv_enc so the crypto path is shared. */
struct sw_dn_enc {
	uint16_t company_id;   /* = SW_COMPANY_ID                         */
	uint8_t  proto_ver;    /* = SW_PROTO_VER                          */
	uint16_t wgn_group;    /* isolation group (from wagon number)     */
	uint8_t  node_type;    /* target node type                        */
	uint8_t  node_id;      /* target node id                          */
	uint32_t ctr;          /* monotonic downlink counter (replay bar) */
	uint8_t  ct[6];        /* AES-CCM ciphertext of sw_dn_pt          */
	uint8_t  mic[4];       /* AES-CCM authentication tag              */
} __attribute__((packed));

/* ---- bulk image transfer (sub-node OTA) ----------------------------------
 *
 * The command frame above carries SIX bytes of ciphertext - enough for a
 * threshold, nowhere near enough for a ~190 KB firmware image. Image data
 * therefore rides its OWN characteristic with its own, larger sealed frame.
 *
 * WHY NOT SMP/MCUmgr: Zephyr's standard DFU transport would give chunking and
 * resume for free, but it is unauthenticated at the application layer - anything
 * able to reach the service could push firmware. Everything else on this link is
 * sealed under the per-wagon key, and the update path is the LAST place to make
 * an exception. So bulk data reuses the same AES-CCM construction.
 *
 * Nonce domain 0x02 (SW_DIR_IMG) keeps image frames apart from uplink adverts
 * (0x00) and config writes (0x01). All three share one key and each has its own
 * counter, so without separate domains a chunk with ctr=N and an advert with
 * ctr=N would produce an IDENTICAL nonce - and CCM nonce reuse leaks keystream
 * and permits forgery.
 */
#define SW_IMG_CHUNK_MAX  192       /* payload bytes per frame */

/* Plaintext image chunk. `len` may be < SW_IMG_CHUNK_MAX only on the LAST
 * chunk; the receiver checks that so a short frame cannot silently truncate an
 * image and still pass verification. */
struct sw_img_pt {
	uint32_t offset;                 /* byte offset into the image */
	uint16_t len;                    /* valid bytes in data[]      */
	uint8_t  data[SW_IMG_CHUNK_MAX];
} __attribute__((packed));

/* On-air image frame. Same header shape as sw_dn_enc so the address and replay
 * checks are identical; only the sealed body is bigger. */
struct sw_img_enc {
	uint16_t company_id;
	uint8_t  proto_ver;
	uint16_t wgn_group;
	uint8_t  node_type;
	uint8_t  node_id;
	uint32_t ctr;                    /* image counter - own replay bar */
	uint16_t ct_len;                 /* ciphertext bytes that follow   */
	uint8_t  ct[sizeof(struct sw_img_pt)];
	uint8_t  mic[4];
} __attribute__((packed));

/* 128-bit GATT service + characteristics for the downlink channel.
 *   service:  5357444E-0001-4A5A-9C36-1D2B3C4D5E6F   ("SWDN")
 *   char cfg: 5357444E-0002-4A5A-9C36-1D2B3C4D5E6F   (write, sw_dn_enc)
 *   char img: 5357444E-0003-4A5A-9C36-1D2B3C4D5E6F   (write, sw_img_enc)
 * Declared here so gateway and node cannot drift apart. */
#define SW_DN_SVC_UUID_VAL \
	BT_UUID_128_ENCODE(0x5357444e, 0x0001, 0x4a5a, 0x9c36, 0x1d2b3c4d5e6f)
#define SW_DN_CHR_UUID_VAL \
	BT_UUID_128_ENCODE(0x5357444e, 0x0002, 0x4a5a, 0x9c36, 0x1d2b3c4d5e6f)
#define SW_IMG_CHR_UUID_VAL \
	BT_UUID_128_ENCODE(0x5357444e, 0x0003, 0x4a5a, 0x9c36, 0x1d2b3c4d5e6f)

#endif /* SENSOR_PROTO_H */
