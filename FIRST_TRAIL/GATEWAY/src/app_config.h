/* Gateway configuration - edit per deployed wagon. */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ---- identity (single source of truth: the wagon number) ----
 * Everything is derived from WAGON_NUMBER, so provisioning a wagon is ONE
 * value:
 *   - {wgn} topic key      = WAGON_NUMBER
 *   - {gw}  / MQTT client  = "GW-" + WAGON_NUMBER
 *   - BLE isolation group  = sw_wgn_group(WAGON_NUMBER)  (CRC16, at runtime)
 *   - per-wagon BLE key     = HKDF(master, WAGON_NUMBER) (sw_secure, runtime)
 * Every sensor node on this wagon MUST use the same WAGON_NUMBER.
 */
/*
 * FACTORY SEED for the wagon number - NOT the live value.
 *
 * config.c copies this into FRAM only when FRAM holds nothing valid for the
 * current CFG_VER. After that the FRAM copy is authoritative, so an OTA
 * carrying a different default does NOT change a commissioned wagon's
 * identity. Read the live value with cfg_wagon() / cfg_gw_id() from config.h;
 * never use this macro directly outside config.c.
 *
 * The old WAGON_NUM and GW_ID macros are gone on purpose. They were
 * compile-time string concatenations, which is exactly what tied identity to
 * the image and made a single fleet-wide .bin impossible.
 */
#define WAGON_NUMBER "31054312345"      /* RDSO §7.19 seed only */
#define FW_VERSION   "0.0.1"

/* MQTT telemetry protocol version (SmartWagon Telemetry Protocol Rev.1 -> 1).
 * This is DISTINCT from SW_PROTO_VER (the BLE advert format version) - do not
 * conflate them. It is the "pv" field in every uplink envelope. */
#define PROTO_PV     1

/* ---- gateway battery (rechargeable LTO pack, read via BQ25798 - charger.c) --
 * The gateway is NOT the 3.6 V primary cell used by the subnodes; it is a
 * rechargeable LTO pack (nominal ~4.8 V for a 2S stack) managed by the BQ25798.
 * LTO has a sloped discharge, so a voltage->% map is meaningful. Set the pack's
 * empty/full points below (measure on the real pack to tune). */
/*
 * FACTORY SEED for the pack's SoC map - NOT the live values.
 *
 * config.c copies these into the stored config only when nothing valid is held
 * for the current CFG_VER; after that the stored pair wins, so an OTA carrying
 * different defaults does not re-tune a pack somebody has already measured.
 * Read them with cfg_batt_full_mv() / cfg_batt_empty_mv(); set them over the
 * air with dn/cmd set_batt.
 *
 * The pack is LTO2S4P (schematic sheet 2) - two lithium-titanate cells in
 * series, four parallel, with a 2S balancer. Per cell: 2.4 V nominal, 2.8 V
 * absolute charge limit, ~1.8 V practical floor. So x2 for the pack.
 *
 * 5400 = 2.70 V/cell. Deliberately BELOW the 2.8 V/cell limit, because 100 %
 * has to be REACHABLE: if the charger terminates below this value the gateway
 * never reports a full pack and every wagon in the fleet looks like it has a
 * failing solar panel. Erring low costs a little resolution at the top; erring
 * high manufactures a fleet-wide false fault.
 *
 * 4000 = 2.00 V/cell. Above the 1.8 V/cell chemistry floor on purpose - the
 * pack never gets that low, because the power mux hands over to the Li-SOCl2
 * backup at around 4.1 V. 0 % should mean "the LTO has stopped being useful",
 * not "the cells are about to be damaged". This choice also orders the alarms
 * correctly: LTO_SWITCHOVER_PCT (15 %) lands at ~4210 mV and GW_LOWBATT_PCT
 * (10 %) at ~4140 mV, so the warning fires before the handover rather than
 * after it.
 *
 * MEASURE RATHER THAN TRUST THESE. Charge until the BQ25798 reports chg:done
 * and read lto.v -> that is full. Run down until src switches to bkp and read
 * lto.v -> that is empty. Then set both with dn/cmd set_batt.
 */
#define GW_BATT_FULL_MV   5500   /* 2S LTO design maximum                     */
#define GW_BATT_EMPTY_MV  3800   /* just above the mux handover - see below    */

/*
 * Acceptance window for dn/cmd set_batt, millivolts.
 *
 * Physically possible limits for a 2S LTO pack, not a preference: 3000 is
 * 1.50 V/cell (below any usable LTO) and 5800 is 2.90 V/cell (above the
 * chemistry's charge limit). A value outside this is a typo or a unit mix-up,
 * and applying it corrupts every battery reading the wagon reports for the rest
 * of its service - an empty_mv set too high reports a healthy pack as flat and
 * triggers LOW_BATTERY on the whole fleet; too low hides a genuinely dying one.
 *
 * The 500 mV span floor keeps the map from collapsing: with the two ends closer
 * than that, one millivolt of ADC noise moves the reported percentage by whole
 * digits, and equal ends would divide by zero.
 */
/*
 * FACTORY SEED for the gateway's BLE output power, dBm.
 *
 * Read the live value with cfg_ble_tx_dbm(); set it with dn/cmd set_ble_tx.
 * This governs the gateway's TRANSMIT side only - the connection it opens to a
 * node during that node's config window, carrying thresholds and OTA. It has
 * NO effect on the RSSI the gateway reports for node adverts, which measures
 * the NODE's transmitter.
 *
 * +8 dBm (the nRF54L15 ceiling) is affordable here in a way it is not on a
 * node: the gateway runs from a solar-fed rechargeable LTO pack and has no
 * six-year primary-cell budget to protect.
 */
#define GW_BLE_TX_DBM     8

/*
 * 3000 = 1.50 V/cell on the 2S LTO pack - below anything usable, so nothing
 * legitimate is excluded. It was briefly 2500 to allow a single bench cell to
 * be calibrated over the air; with GW_BATT_EMPTY_MV back at 3800 that headroom
 * is no longer needed, and a tighter floor is the better default for the
 * hardware that actually ships.
 *
 * Note this is only the ACCEPTANCE bound for dn/cmd set_batt, not a target. If
 * a bench cell ever needs calibrating below 3000 mV again, widen it here rather
 * than working around the validator.
 */
#define GW_BATT_MV_MIN    3000
#define GW_BATT_MV_MAX    5800
#define GW_BATT_MV_SPAN   500

/* ---- broker --------------------------------------------------------------
 * The ADDRESS and CREDENTIALS live in secrets.h, which is gitignored, so no
 * password ever enters the repository. Everything else about the broker is
 * public and stays here.
 *
 * secrets.h is OPTIONAL: __has_include() detects it, and the fallbacks below
 * take over when it is absent, so a fresh clone compiles and runs against the
 * anonymous public broker without any credential. To provide one, copy
 * secrets.h.example to secrets.h and fill it in.
 */
#if defined(__has_include)
#  if __has_include("secrets.h")
#    include "secrets.h"
#  endif
#endif

#ifndef MQTT_HOST
#define MQTT_HOST      "mqtt.macnman.com"   /* anonymous fallback */
#endif
#ifndef MQTT_USERNAME
#define MQTT_USERNAME  ""
#endif
#ifndef MQTT_PASSWORD
#define MQTT_PASSWORD  ""
#endif

/* An empty MQTT_USERNAME makes ec200.c issue the 2-argument AT+QMTCONN and send
 * no credentials at all, so the fallback is a working anonymous connection
 * rather than a failed authenticated one. */
/* MQTT client id is cfg_gw_id() - runtime, from FRAM. See config.h. */
/*
 * APN for the SIM in this gateway. Unlike the MQTT settings above this had NO
 * #ifndef guard, so a value in secrets.h was silently ignored and every unit
 * had to be edited in tracked source - exactly what secrets.h exists to avoid.
 *
 * An empty APN means the modem never attaches: ec200.c issues AT+QICSGP with an
 * empty context and the PDP activation fails, so MQTT never gets a carrier.
 * Common Indian M2M values: "airtelgprs.com", "www" (Vi), "jionet", "bsnlnet".
 * An eSIM profile usually carries its own - ask the provider rather than guess.
 */
/*
 * AIRTEL, read off the module rather than guessed: all_hw_test on this board
 * reported +COPS: 0,0,"AIRTEL",7 and published to the broker with this APN.
 *
 * A DIFFERENT SIM WILL NEED A DIFFERENT VALUE. M2M SIMs frequently carry a
 * private APN rather than the consumer one, so treat this as the value that
 * works for the SIM currently fitted, not a fleet default. Setting it empty is
 * legitimate and means "use the bearer the network assigns" - ec200.c skips
 * QICSGP entirely in that case rather than sending an empty string, which the
 * module rejects.
 */
#ifndef CELL_APN
#define CELL_APN       "airtelgprs.com"
#endif

/*
 * ---- transport security (RDSO s3.8) ----
 *
 * s3.8 requires platform data to be "encrypted by the latest security standards
 * available". Plain MQTT on 1883 puts every reading, alarm and downlink command
 * on the cellular network in clear text, so TLS is ON by default here.
 *
 * MQTT_TLS_SECLEVEL:
 *   0 = encrypt only, DO NOT verify the server. Stops passive eavesdropping but
 *       NOT an active man-in-the-middle, so it does not really satisfy s3.8.
 *       Bench use only.
 *   1 = verify the server against MQTT_TLS_CA_FILE.  <-- production
 *   2 = mutual auth; also needs a client cert + key provisioned.
 *
 * BEFORE FIRST USE the CA certificate must be uploaded to the modem ONCE:
 *   AT+QFUPL="ca.pem",<len>,100      then send the PEM bytes
 * The firmware deliberately does not upload it on every connect - that would
 * wear modem flash for no benefit.
 *
 * Set MQTT_TLS to 0 ONLY for bench work against a broker with no TLS listener.
 * Doing so is a KNOWN DEVIATION from s3.8 and must not ship.
 *
 * >>> CURRENTLY 0: KNOWN DEVIATION FROM RDSO s3.8. <<<
 * The broker has no TLS listener yet, and enabling TLS before it does would
 * stop the gateway connecting at all - QMTOPEN fails, nothing is published, and
 * downlink commands pile up at the broker. Set this back to 1 once
 * mqtt.macnman.com accepts TLS on 8883 AND the CA certificate has been uploaded
 * to the modem. Nothing else needs to change: the port and the whole AT
 * sequence follow this switch.
 */
#define MQTT_TLS               0
#define MQTT_TLS_SECLEVEL      1
#define MQTT_TLS_SECLEVEL_STR  "1"      /* must match MQTT_TLS_SECLEVEL */
#define MQTT_TLS_CA_FILE       "ca.pem"

/*
 * TLS for the HTTPS firmware download (SSL context 1) - separate from MQTT's
 * context 2, because the broker and the firmware host are different servers
 * with different certificate authorities.
 *
 *   0 = encrypt but do NOT verify the server.  <-- bring-up / demo
 *   1 = verify against OTA_TLS_CA_FILE.        <-- production
 *
 * 0 is a smaller compromise here than it would be for MQTT. The image is
 * MCUboot-signed, so an attacker who impersonated the host still cannot make
 * the gateway RUN anything - the signature check rejects it. What level 0
 * gives up is confidentiality of a firmware build you are shipping anyway, and
 * protection against being fed an old but validly signed image.
 *
 * Level 1 needs the host's CA uploaded to modem flash once via AT+QFUPL. For
 * raw.githubusercontent.com that is GitHub's current DigiCert chain - confirm
 * it rather than assuming, since a rotated CA breaks every update at once.
 */
#define OTA_TLS_SECLEVEL       0
#define OTA_TLS_SECLEVEL_STR   "0"      /* must match OTA_TLS_SECLEVEL */
#define OTA_TLS_CA_FILE        "ota_ca.pem"

#if MQTT_TLS
#define MQTT_PORT      8883
#else
#define MQTT_PORT      1883
#endif

/* Topic base: smartwagon/v1/<wgn>/<gw>/...  (built in telemetry.c) */
#define TOPIC_ROOT   "smartwagon/v1"

/* ---- fleet addressing: group-wise and bulk downlink (RDSO s5.1.13) -------
 * s5.1.13 requires the FUOTA mechanism to accept "individual, group wise or
 * bulk upgrade commands". This is done WITHOUT inventing any topic.
 *
 * Protocol Rev.1 s7.1 defines exactly one downlink topic:
 *
 *   smartwagon/v1/{wgn}/{gw}/dn/cmd
 *
 * The gateway subscribes to TWO topics (see ec200.c):
 *
 *   smartwagon/v1/{wgn}/{gw}/dn/cmd   individual - this wagon only
 *   smartwagon/v1/all/dn/cmd          BULK - one publish reaches every gateway
 *
 * The "all" topic is an addition to Rev.1, kept because the alternative for
 * "update every wagon" is 2500 separate publishes and no way to express the
 * intent atomically. It does NOT break s7.2's documented cloud wildcards
 * (smartwagon/v1/+/+/up/#) because those match UPLINK topics, and nothing
 * publishes uplinks under "all".
 *
 * GROUP (rake-wise) addressing is deliberately NOT a topic. A per-rake topic
 * such as smartwagon/v1/grp/<group>/dn/cmd was tried and removed: a literal
 * "grp" in position 3 mis-parses for any subscriber that assumes a wagon
 * number there, and a rake is re-marshalled constantly so any stored rake id
 * goes stale on the next formation. Group scope therefore travels in the
 * PAYLOAD as "scope", and the CLOUD fans it out one publish per wagon -
 * because the cloud is what knows the current composition.
 *
 * NOTE: an earlier version of this comment said the "all" topic had been
 * removed. That has not been true since the bulk subscription was added; the
 * text described a revision that no longer exists.
 *
 * What "scope" still changes on the gateway: a group or bulk ota_start is
 * STAGGERED (below) and is a no-op when the wagon already runs the named
 * version. Everything else behaves identically however it was addressed.
 *
 * Responses ALWAYS go to this gateway's own up/resp, so the back office can
 * tell which wagons acted on a fan-out and which never answered. */

/* NEVER publish a command with RETAIN set. A retained command is re-delivered
 * on every reconnect, which for ota_start means the wagon re-downloads the
 * image on every wake, forever. The version guard in ota_start() is the
 * backstop, not a licence to retain. */

/* Stagger window for a GROUP or BULK ota_start, milliseconds.
 * A broadcast reaching 2500 wagons at once would have all of them open an HTTP
 * download of the same image in the same second - a self-inflicted DDoS on the
 * firmware host, and a burst the cellular network will shed. Each gateway
 * delays by a DETERMINISTIC offset derived from its wagon number, so the fleet
 * spreads itself evenly across this window without any central scheduling and
 * without two wagons ever colliding on the same slot by chance.
 * An INDIVIDUAL ota_start is never staggered - an operator asking one wagon to
 * update expects it to start now. */
#define OTA_FLEET_STAGGER_MS  (10UL * 60UL * 1000UL)   /* 10 min spread */

/* ---- cadence (RDSO §7.4) ---- */
/* ---- bench debug switch ------------------------------------------------
 * ONE flag, named to match DEBUG_TRACE on the sub-nodes so the whole project
 * has a single thing to turn off. Set to 0 for production.
 *
 * Turning it on shortens the reporting cadence to something a person can watch
 * and prints a loud banner at boot, so an image built this way cannot be
 * mistaken for a shippable one.
 */
/*
 * Bisection switch, temporary.
 *
 * 1 = do not touch NRF_CE, MUX_ST, LC_WKP or CHG_INT - the four pins the
 * working all_hw_test never mentions and this application drives. Two of them
 * sit in the power path that feeds the modem's VBAT.
 *
 * Set back to 0 once the cause is identified; the charger enable and the status
 * inputs are real functionality, not decoration.
 */
/*
 * RESULT: 0. Those four pins are NOT the fault - and NRF_CE matters.
 *
 * With them left undriven the NETLIGHT stopped blinking entirely, where before
 * it blinked after a PWRKEY pulse. NRF_CE low keeps the BQ25798 charging, and
 * without it the system rail cannot support the modem starting at all. So the
 * experiment made things strictly worse, which exonerates the pins and puts the
 * charger enable back where it belongs.
 */
#define MODEM_PINS_EXPERIMENT  0

#define DEBUG_TRACE   1

/*
 * Hold the modem session up between reports. Requires DEBUG_TRACE.
 *
 * DEFAULT 0, matching the DK bench where a 30 s cadence was verified working
 * with the normal power-cycle-per-report behaviour. Attach on this modem is
 * evidently fast enough, so there is no reason to change what already works -
 * and keeping the cycle identical to production means the bench exercises the
 * real code path.
 *
 * Set to 1 only if attach times on the custom board turn out to eat the
 * cadence, or when testing downlink delivery latency: a held session keeps the
 * broker subscriptions alive continuously, so commands land within seconds
 * rather than waiting for the next online window.
 */
/*
 * MODEM_HOLD_SESSION is GONE.
 *
 * It kept the cellular session attached between reports so a bench run did not
 * re-attach every cycle. That was a debugging crutch for a transport that was
 * broken for an unrelated reason - app.overlay had the uart22 TX/RX pins
 * reversed - and with that fixed it only costs power: s5.1.20's six-year life
 * assumes the modem is OFF between 10-minute reports, and a held LTE session is
 * the largest continuous load on the board.
 *
 * Removed rather than defaulted to 0, because a switch that must never be 1 in
 * production is a switch that eventually ships as 1.
 */

#if DEBUG_TRACE
/*
 * BENCH CADENCE - NOT SPEC COMPLIANT.
 *
 * s7.4 requires cloud communication every 10 minutes in normal working
 * conditions, and s5.1.20 builds the 6-year battery life on that number.
 * These values exist only so a bench session produces visible traffic.
 *
 * 30 s / 1 min are the MINIMUM values set_interval will accept (it validates
 * moving_s >= 30, idle_s >= 60), so the bench runs at the same floor an
 * operator could reach over the air.
 *
 * The modem is still power-cycled for every report here, exactly as in
 * production. That combination was verified working on the DK bench, so there
 * is no reason to special-case it; a held session would exist if attach
 * times ever do become the bottleneck.
 */
#define INTERVAL_MOVING_MS  (60UL * 1000UL)          /* 1 min - BENCH ONLY */
#define INTERVAL_IDLE_MS    (120UL * 1000UL)         /* 2 min - BENCH ONLY */
#else
#define INTERVAL_MOVING_MS  (10UL * 60UL * 1000UL)   /* 10 min running */
#define INTERVAL_IDLE_MS    (12UL * 60UL * 60UL * 1000UL) /* 12 h stopped */
#endif
#define MOVING_SPEED_KMH    3.0
#define VIBRATION_MIN_KMH   15.0        /* bearing vibration valid above this */

/* Longest fast config window a node_window command may request, seconds.
 * MUST NOT EXCEED the sub-nodes' own CFGWIN_FAST_MAX_S - the node caps it
 * anyway, but rejecting here gives the operator a clear BAD_PARAM instead of
 * silently getting a shorter window than they asked for.
 *
 * Fast mode is ~6.4 mA on the node (4 s connectable in every 5 s), so an hour
 * costs ~6.4 mAh - under 0.1 % of a 10,000 mAh cell. Bounded on purpose: the
 * node reverts on its own deadline and never persists the state, so no command
 * from here can leave a node burning battery unattended. */
#define CFGWIN_FAST_MAX_REQ_S   3600

/* ---- critical-alarm receive window (protocol §3.6) ---- */
#if DEBUG_TRACE
/*
 * BENCH SCAN DUTY - power-hungry, do not ship.
 *
 * The production figures below are 100 ms every 5 s: a 2 % duty cycle, sized so
 * a node that has escalated to its 1 s ALARM cadence (300 ms of advertising
 * every second, ~30 % duty) is heard within a few seconds. That is the case
 * §3.6 actually cares about.
 *
 * A HEALTHY node is a different matter. It advertises 300 ms once every 30 s -
 * roughly 1 % duty - and two low-duty cycles rarely coincide: a 300 ms burst
 * overlaps a 100 ms window in only about 8 % of cycles, so first contact
 * averages several MINUTES. Nothing is broken while you wait, but on a bench
 * it looks broken, and the temptation is to go hunting for a fault that is not
 * there.
 *
 * 200 ms every 250 ms (80 %) means a scan window ALWAYS falls inside any 300 ms
 * burst, so a healthy node is heard on its very first advertisement.
 */
#define RX_WINDOW_MS        200
#define RX_INTERVAL_MS      250
#define NODE_STALE_MS       (2UL * 60UL * 1000UL)//node stale timing 2 min 
#else
#define RX_WINDOW_MS        100
#define RX_INTERVAL_MS      5000
#define NODE_STALE_MS       (15UL * 60UL * 1000UL)//node stale timing 15 min 
#endif

/* Grace period before a node that has NEVER been heard is declared down.
 * Added on top of NODE_STALE_MS and applied ONLY to never-heard nodes, so a
 * cold-started gateway does not fault its whole roster before the sub-nodes
 * have had a realistic chance to advertise. A node that has been heard once is
 * unaffected - a genuine dropout is still caught on NODE_STALE_MS.
 * Sized at a few sub-node advert periods (they advertise every 30 s). */
#define NODE_FIRST_HEARD_GRACE_MS   (2UL * 60UL * 1000UL)   /* 2 min */

/* Gap inserted between consecutive alarm publishes. Each QoS 1 publish needs a
 * PUBACK round trip; firing a whole roster back to back is what kills the link
 * (+QMTPUB: 0,N,1,1 followed by +QMTSTAT: 0,8). Applied only on a real state
 * CHANGE, so a healthy wagon with every node present never pays it.
 * 19 nodes x 250 ms = ~5 s worst case, well inside the report cycle. */
#define ALARM_BURST_GAP_MS          250

/* report-time active BLE scan boost (short -> negligible power) */
#define BLE_REFRESH_MS      2000

/* ---- motion debounce (persistence filter) ----
 * A raw motion change must persist this long before it is CONFIRMED. This is
 * what makes a 1-min shunt (start-then-stop) or a 1-min mid-run pause NOT flip
 * the cadence or fire a TRAIN_START/TRAIN_STOP event.
 *   START_CONFIRM: stopped -> moving must hold this long  (2 min)
 *   STOP_CONFIRM : moving  -> stopped must hold this long (5 min)
 * While a transition is pending (or moving) the gateway re-samples the BMA400
 * every MOTION_POLL_MS to time it. */
#define MOTION_START_CONFIRM_MS  (2UL * 60UL * 1000UL)   /* 2 min sustained */
#define MOTION_STOP_CONFIRM_MS   (5UL * 60UL * 1000UL)   /* 5 min sustained */
#define MOTION_POLL_MS           (30UL * 1000UL)         /* re-sample cadence */

/* BMA400 burst-variance threshold (g^2). Above this = vibration = moving.
 * A still wagon reads ~1 g with near-zero variance; tune on hardware. */
#define MOTION_ACC_VAR           0.0009

/* Impact/shock alarm magnitude (g). |a| above this at an EV_IMPACT wake is
 * reported as a shock/impact alarm (RDSO impact criteria - tune to profile). */
#define IMPACT_G_THRESH          4.0

/* ---- gateway-level alarm/event thresholds (protocol Rev.1 s3.2 / s4.2) ----
 * These drive gwalarm.c. All are provisional engineering values and must be
 * confirmed against RDSO WD-35-MISC-2024 s7.10 (bands) and s7.15 (thresholds)
 * before certification; several are runtime-settable via set_threshold. */

/* DERAIL vs IMPACT (s5.3.3). A shock this large WHILE MOVING is a derailment
 * candidate; the same shock while stopped is yard handling. */
#define DERAIL_G_THRESH          8.0

/* Bearing condition bands (s7.10), degC. */
#define BAND_YELLOW_C            70
#define BAND_RED_C               95

/* Cargo over-temperature threshold reported in the TANK_OVERTEMP alarm's "thr"
 * field, degC. The TANK NODE owns the actual decision - it raises SW_FLAG_ALARM
 * from its own ALARM_THRESHOLD (600 = 60.0 C) with ALARM_HYST release - so this
 * is the gateway telling the cloud what limit was applied, not a second test.
 * KEEP IN SYNC with ALARM_THRESHOLD in each tank node's app_config.h. */
#define TANK_OVERTEMP_C          60.0

/* Wheel flat / RCF: vibration index from the bearing node (value2). */
#define FLAT_WHEEL_VIB_THRESH    800

/*
 * Yellow band for WHEEL condition, raw vibration index (RDSO s7.11).
 *
 * s7.11 wants three bands and only the red one existed - FLAT_WHEEL_VIB_THRESH
 * above, which raises the alarm. Without a yellow there is no "watch this"
 * state, so a wheel goes from Green straight to Red with no warning, defeating
 * the point of trending.
 *
 * PROVISIONAL: 60 % of the red threshold, a placeholder with no measurement
 * behind it. Vibration index units are whatever the bearing node's accelerometer
 * pipeline produces, and that pipeline does not exist yet (every bearing
 * sensor.c is still a stub), so this cannot be calibrated until real nodes run
 * on real track. Revisit with field data before acceptance.
 */
#define WHEEL_VIB_YELLOW         480

/*
 * LTO pack nominal capacity, mAh - used only for pwr2.aut (autonomy hours).
 *
 * CONFIRM AGAINST THE FITTED PACK. The 2S4P arrangement means capacity is four
 * cells in parallel; this figure is a placeholder until the actual cell part
 * number is known. aut is reported null whenever this cannot be turned into a
 * meaningful number, which is better than publishing a confident wrong one.
 */
#define GW_BATT_CAPACITY_MAH     8000

/* Load (s5.3): overload limit and the delta that counts as load/unload. */
#define OVERLOAD_THRESH          95
#define LOAD_CHANGE_DELTA        20

/* Battery advisories. Node cells are non-replaceable (s7.14). */
#define NODE_LOWBATT_PCT         15
#define GW_LOWBATT_PCT           10

/* Power: current INTO the pack that counts as charging, and the LTO SoC at
 * which the mux is expected to have fallen over to the Li-SOCl2 backup. */
#define CHARGE_I_THRESH_MA       20
/*
 * Pack voltage at which the power mux is taken to have handed over, mV.
 *
 * A VOLTAGE, not a percentage. src used to be inferred from state-of-charge,
 * which meant a mis-set full/empty map produced a wrong src: on the bench a
 * 3.7 V supply read 0 % against a 2S map and the gateway reported running on a
 * backup cell that was not even fitted. Voltage is the quantity the TPS2116
 * itself decides on, so comparing against a voltage cannot drift with the
 * SoC calibration.
 *
 * COMPUTED FROM THE HARDWARE, not chosen. The TPS2116's PR divider is R31
 * 220k from VIN1 and R35 100k to GND, so the pin sees VIN1 x 0.3125, and the
 * comparator trips at V_PR ~1.18 V:
 *
 *     VIN1(switchover) = 1.18 / 0.3125 = 3.78 V
 *
 * Firmware does not decide this and must not try - the mux switches in
 * microseconds from its own threshold, while the CPU is usually asleep. This
 * constant exists only so the REPORT agrees with what the hardware already did.
 *
 * GW_BATT_EMPTY_MV sits just above it (3800) on purpose: 0 % then means "the
 * mux is about to hand over to the backup", which is when the LTO has actually
 * stopped being useful - not the chemistry's damage floor. The two numbers are
 * tied together and should move together.
 *
 * CONFIRM V_PR AGAINST THE DATASHEET before trusting the third digit. 1.18 V is
 * the typical figure; the divider ratio is exact.
 */
#define LTO_SWITCHOVER_MV        3780

/*
 * LTO_SWITCHOVER_PCT is GONE.
 *
 * It was a state-of-charge PERCENTAGE, and deriving `src` from it meant a
 * mis-set full/empty map produced a wrong answer: on the bench a 3.7 V supply
 * read 0 % against a 2S map, and the gateway reported running on a backup cell
 * that was not fitted. The mux decides on VOLTAGE, so the report has to compare
 * a voltage - see LTO_SWITCHOVER_MV above.
 *
 * Deleted rather than left unused. A dead constant that looks like it still
 * governs behaviour is worse than no constant: the next person to tune the
 * switchover would change this one and see nothing happen.
 */

/*
 * TPS2116 ST-pin polarity: what level means "VIN1 (LTO) is selected".
 *
 * ST is an OPEN-DRAIN status output, so it is read with a pull-up and the
 * asserted state is LOW. On the TPS2116 that assertion indicates VIN1, and
 * LTO_BAT is wired to VIN1 on this board - hence 0 here.
 *
 * SET THIS FROM THE FIRST BOOT LOG, DO NOT TRUST IT BLIND. power_mux_init()
 * prints the raw pin level together with the interpretation it produced. The
 * gateway is running from LTO whenever you can read that line at all - a board
 * on a backup that is not fitted would be dead - so if the log says "bkp" while
 * the console is alive, this constant is inverted. Getting it backwards is
 * worse than the inference it replaces, because a wrong measurement carries the
 * authority of a real one.
 */
/*
 * CONFIRMED ON HARDWARE: ST reads 1 while the board runs from LTO.
 *
 * The first boot printed "MUX: ST=1 -> bkp selected" on a gateway with NO
 * backup cell fitted - which was running, and therefore on LTO by definition.
 * A board on an absent backup would be dead. So ST high = VIN1 (LTO) selected
 * on this part, and the initial 0 was wrong.
 */
#define MUX_ST_LTO_LEVEL          1

/*
 * Read the mux status pin at all? 0 = leave P1.11 COMPLETELY untouched.
 *
 * DEFAULT OFF, and deliberately so. The gateway ran for 2400+ s without a reset
 * until the build that began configuring this pin; from then on every boot
 * reported RESET: POWER. That is only a correlation - the same builds also
 * lengthened the modem attach window and added AT+CSQ / AT+CEREG traffic, so a
 * marginal supply has another candidate - but this is the only change that
 * touches hardware, and it is the cheapest one to eliminate.
 *
 * The risk is concrete: board_pins.h calls P1.11 "power-mux status/select", and
 * nobody has confirmed which. If it is the TPS2116's MODE (select) rather than
 * its ST (status), then enabling a pull-up on it can change which rail the mux
 * presents - and browning the board out is exactly what that would look like.
 *
 * Set to 1 ONLY to test the hypothesis, or once the schematic has confirmed
 * P1.11 is ST. With it at 0, pwr.src falls back to the state-of-charge estimate
 * and Rev.1 s2.2 ("read from the power-mux status, not inferred") stays unmet -
 * a known deviation, and a far smaller problem than a rebooting gateway.
 */
#define MUX_ST_READ_ENABLE        0

/*
 * How long to let the modem's VBAT rail settle before pulsing PWRKEY, ms.
 *
 * The first PWRKEY pulse of every cycle FAILS - "MS: modem did not answer AT" -
 * and only the second starts the module. That is not a toggle-ordering bug: if
 * the first pulse had switched the module ON, the second would switch it back
 * OFF, and it would never work. The first pulse simply is not being seen.
 *
 * The reason is the rail. modem_power_on() raises SW_GNSS and reaches PWRKEY
 * about 140 ms later - but the EC200U's bulk capacitance is still charging
 * through the TPS63802, and on a marginal supply VBAT is still below the
 * module's minimum when PWRKEY arrives. By the second attempt the rail has been
 * up for 20+ seconds, the caps are charged, and it starts immediately.
 *
 * Waiting costs nothing and removes an entire power-on surge per cycle - which
 * matters on a supply that is browning the board out at PDP activation.
 *
 * This is a MITIGATION, not the fix. A rail that needs most of a second to
 * reach the modem's minimum is under-provisioned; see RESET: POWER at boot.
 */
#define MODEM_VBAT_SETTLE_MS      800

/* Geofence (s7.5). Set GEOFENCE_RADIUS_M to 0 to disable. A single circular
 * zone only - the protocol defines the EVENTS but no command to provision
 * zones, so multi-zone support needs a spec extension. */
#define GEOFENCE_LAT             0.0
#define GEOFENCE_LON             0.0
#define GEOFENCE_RADIUS_M        0

#endif /* APP_CONFIG_H */

/*
 * How long a node stays in fast-window mode during an OTA campaign, in seconds.
 *
 * Fast mode is what makes a sub-node update finish in minutes instead of a day:
 * windows every few seconds instead of every CFGWIN_PERIOD_MS (10 min). It is
 * expensive - the node is advertising connectably for a large fraction of the
 * time - so it is DEADLINED rather than toggled. If the gateway dies mid
 * campaign, or the final "cancel" is lost, the node reverts on its own.
 *
 * 1800 s covers a full image with margin at the observed chunk rate. Raising it
 * costs battery directly; lowering it risks a campaign that never converges
 * because fast mode expires between windows.
 */
#define NODE_OTA_FASTWIN_S   1800

/* ---- Thermal transmit halt (RDSO s5.1.10) -------------------------------
 * The spec mandates the CAPABILITY but sets no number - s7.15 makes threshold
 * values the vendor's responsibility, to be agreed with Indian Railways.
 *
 * Derived from s5.1.9, which requires reliable operation over -20..+85 C:
 * transmitting above +85 C means running outside the specified envelope, so
 * that is the halt point. The 10 C release band is deliberately wide - each
 * resume attempt is itself a heat pulse, so a narrow band would produce a
 * gateway that oscillates and never actually cools.
 *
 * Data capture continues while halted; records buffer to the NOR ring. A
 * thermal event therefore costs latency, never data.
 */
#define THERMAL_HALT_C     85   /* halt TX at or above this die temperature */
#define THERMAL_RESUME_C   75   /* resume only after cooling to this        */
