# Smart Wagon — MQTT Uplink Schema (reader's guide)

How to consume telemetry from a **fleet of wagons**, each with **multiple bogies
and sensors**, with the least parsing effort. SmartWagon Telemetry Protocol
Rev.1 (`pv=1`). Controller nRF54L15 (QFN52).

---

## 1. How multiple wagons stay separate (you get this for free)

Every uplink is isolated three ways, so a reader never has to disentangle wagons:

1. **Topic** carries the wagon and gateway:
   `smartwagon/v1/{wgn}/{gw}/up/{hb|alarm|event|resp}`
   Subscribe with wildcards and route by the topic level, e.g.
   `smartwagon/v1/+/+/up/hb` for all heartbeats, or
   `smartwagon/v1/31054312345/+/up/#` for one wagon.
2. **Payload is self-contained** — every message repeats `wgn` and `gw` in the
   body, so a row written to a database needs no topic parsing.
3. **`seq`** is a per-gateway monotonic counter — de-duplicate and order on
   `(gw, seq)`. (Offline messages are replayed later with their **original**
   `seq` and `ts`, so dedup on `(gw,seq)` is what makes replay idempotent.)

**One primary key for everything:** identify a sensor by **`(wgn, typ, pos)`** —
NOT by `node_id`. `pos` (e.g. `B1-A1-L`) is position-semantic and means the same
thing on every wagon, whereas `node_id` is only a wagon-local, fitment-dependent
address that can map to a different sensor on a different wagon. Add `seq` for a
specific reading. (See `NODE_MAP.md` for the canonical fleet id allocation and
why removing a node never renumbers the others.)

## 2. Common envelope (every message starts with these)

```json
{
  "mt": "hb",              // message type: hb | alarm | event | resp
  "pv": 1,                 // telemetry protocol version (NOT the BLE version)
  "seq": 4213,             // per-gateway monotonic; dedup/order key
  "gw": "GW-31054312345",  // gateway id
  "wgn": "31054312345",    // wagon number  <-- primary wagon key
  "ts": 1786060800,        // UTC epoch seconds (from GNSS)
  "loc": { "lat": 19.076, "lon": 72.8777, "cep": 4, "sys": ["navic","gps"] },
  //  cep = CEP50 accuracy in metres; 99 = NO USABLE FIX, not a bad fix.
  //  sys = constellations that PRODUCED the fix; [] when there is no fix.
  //        Gate on `fix != "none"`, never on cep alone.
  "spd": 42,               // km/h
  "pwr": { "src":"lto","soc":78,"sol":"charging","bkp":"ok" },
  "d": { ...type-specific body... }
}
```

> **`pwr.soc` and `sens[].bat` are both percentages and are NOT produced the same
> way.** `pwr.soc` is the gateway's rechargeable LTO pack, mapped linearly from
> pack voltage between two calibration points (`dn/cmd set_batt`) - LTO discharges
> on a slope, so voltage is meaningful. `sens[].bat` is a sub-node's Li-SOCl2
> primary cell, which is flat for ~95 % of its life, so it is coulomb-counted
> instead. Do not compare them, and do not apply one's accuracy expectations to
> the other.

## 3. Heartbeat (`mt:"hb"`) — the per-wagon periodic report

The `d.sens[]` array has **one element per fitted sensor**, so the array length
= that wagon's node count (4, 8, 20…). Each element pinpoints a physical sensor:

```json
"sens": [
  { "node":"NODE-BRG-00", "pos":"B1-A1-L", "typ":"btemp",
    "val":74, "unit":"C", "t":55, "bat":92, "age":6 },
  { "node":"NODE-BRG-01", "pos":"B1-A1-R", "typ":"btemp",
    "val":73, "unit":"C", "t":48, "bat":90, "age":6 },
  { "node":"NODE-LOAD-08","pos":"B1-LOAD","typ":"load",
    "val":81, "unit":"state", "t":0, "bat":88, "age":7 },
  { "node":"NODE-BRG-05", "pos":"B2-A1-R", "typ":"btemp", "miss":1 }
]
```

| Field | Meaning |
|---|---|
| `node` | Node serial, unique **within the wagon** (`NODE-<type>-<id>`). |
| `pos`  | **Physical location** — e.g. `B1-A1-L` = bogie 1, axle 1, left. This is what tells you *which wheel/bogie* without decoding ids. Set per your fitment convention in `nodes.c`. |
| `typ`  | `btemp`, `load`, `hbrake`, `ttemp`, `door`, `brake`. |
| `val` / `unit` | Primary reading + its unit (bearing → °C; state sensors → `state`). |
| `t`    | Secondary reading (e.g. vibration index for bearings). |
| `bat`  | Node battery %. **Coulomb-counted, not voltage-inferred** - the Li-SOCl2 cell holds ~3.6 V almost to end-of-life, so voltage says nothing about charge until the cliff. The node integrates (current x time) per phase; accuracy is that of its calibration constants, which are settable over the air. Expect +/-10..20 %: a "schedule a battery swap" gauge, not a laboratory one. |
| `age`  | Seconds since this node was last heard over BLE. |
| `miss` | `1` = this fitted node was **not heard** this cycle (treat as stale). A missing node emits ONLY `node`, `pos`, `typ`, `miss` — no `val`, `t`, `bat` or `age`, so an absent sensor can never be read as a reading of zero. |

> **`null` means "no measurement", everywhere in this schema.** `val` and `t`
> are independently nullable (on a tank node the RTD and the internal sensor
> fail separately); `band` entries, `gsm` fields and the whole `bkp` object use
> the same convention. Nothing in an uplink carries a placeholder value standing
> in for data that was never taken.

To locate a hot bearing on wagon X: filter `wgn == X`, then read
`sens[].pos` — no id math. To roll up per bogie, group by the `B<n>` prefix of
`pos`.

### 3.1 `band` — condition bands (§7.10/§7.11)

```json
"band": { "brg": ["G","G","R","G","G","G","G","G"],
          "whl": ["G","Y","G","G"] }
```

| Array | Length | Derived from |
|---|---|---|
| `brg` | one per **bearing** (8 on a 4-axle wagon) | bearing **temperature** |
| `whl` | one per **axle** (4) | wheel **vibration index** |

Two things to get right:

- **Position is implicit in the index**, not labelled. `brg[3]` is the fourth
  bearing in roster order. Correlate against `sens[].pos`, which carries the
  position explicitly, rather than hardcoding the mapping — a roster change
  shifts every index.
- **An entry can be `null`**, meaning *no measurement* — the node was never
  heard, or reported no reading this cycle. `null` is **not** healthy.
  Previously these reported `"G"`, so a bearing that was unfitted, flat or out
  of range appeared green. A dashboard that colours from `band` alone must
  handle null explicitly, or read `sens[].miss` alongside it.

An axle takes the **worse** of its two bearing ends' wheel bands — a flat is a
property of the wheelset and either end's accelerometer can see it.

### 3.2 `pwr2`, `gsm`, `buf` — the detailed heartbeat tail

```json
"pwr2": { "src":"lto",
          "lto":{ "v":4810, "soc":74, "chg":"fast" },
          "sol":{ "vin":3620, "i":410, "st":"charging" },
          "bkp":{ "ok":null, "v":null, "rem":null, "use":null },
          "aut":2160 },
"gsm":  { "cell":"1A2F123", "rssi":-71 },
"buf":  { "n":12, "old":31, "new":42 }
```

| Field | Meaning |
|---|---|
| `pwr2.src` | active rail, `lto` \| `bkp`. **Derived from SoC**, not sensed — the board's `MUX_ST` line exists but is not yet read, so a wrong battery calibration produces a wrong `src`. |
| `lto.chg` | charger state: `idle`, `trickle`, `precharge`, `fast`, `taper`, `topoff`, `done` |
| `sol.st` | **solar input** state: `off` \| `present` \| `charging`. `present` ≠ `charging` — in low light or cold the panel can supply VBUS while below its charging threshold. |
| `aut` | estimated autonomy, **hours**, at the present draw. `null` while charging or when the draw is too small to divide by. |
| `gsm.rssi` | signal strength, dBm, from `AT+CSQ`. **`0` means not measured** — 0 is not a legal RSSI, so it cannot be mistaken for a reading. |
| `gsm.cell` | serving cell id. **`""` means the network did not report one.** |
| `buf.n` | records held in the store-and-forward ring |
| `buf.old` / `buf.new` | sequence span of that backlog |

> ⚠ **`bkp` is all `null`, and that is deliberate.** Nothing on this hardware
> measures the Li-SOCl2 backup cell — there is no ADC on that net. Earlier
> firmware published `{"ok":true,"rem":18450}` as literals copied from the
> protocol document's own sample, asserting a healthy reserve on a gateway with
> no backup fitted. `null` says "unknown", which is the truth. This is a
> deviation from Rev.1 §5.3, which expects `ok`/`low`/`crit`; all three would
> be lies.
>
> The compact `pwr.bkp` reports `"na"` for the same reason.

## 4. Alarm (`mt:"alarm"`)

```json
"d": {
  "code":"HOT_BEARING", "sev":"red",
  "node":"NODE-BRG-06", "pos":"B2-A2-L",
  "val":97, "unit":"C", "thr":95,
  "st":"set", "aid":"al-4219", "spd":42
}
```

- `code`: the full Protocol Rev.1 §3.2 enumeration — `DERAIL`, `HOT_BEARING`,
  `FLAT_WHEEL`, `DOOR_UNAUTH`, `HANDBRAKE_MOVING`, `OVERLOAD`, `TILT`,
  `TANK_OVERTEMP`, `IMPACT`, `SENSOR_FAULT`, `TAMPER`, `LOW_BATTERY`,
  `NODE_LOW_BATTERY`. All thirteen are implemented.
  - `DOOR_UNAUTH` = door reported open while the gateway's motion state says the
    train is **moving** (open-while-stopped is heartbeat state, not an alarm).
    Latched per door and re-armed when the wagon stops or the door is next seen
    closed, so one open cannot storm.
  - `TAMPER` = an **impact/shock** was detected on a node, raised regardless of
    motion and regardless of node type. The node serial in `node` says which
    device.

  > ⚠ **`DOOR_TAMPER` and `TANK_TAMPER` no longer exist.** Earlier firmware
  > invented a per-type tamper code, which no parser built to Rev.1 would
  > recognise and which silently excluded the other 17 node types. Both are now
  > the spec's single `TAMPER`.

- **`sev` has FOUR values, in two vocabularies** (Rev.1 §3.1):

  | Value | Used for |
  |---|---|
  | `yellow` | bearing/wheel **condition band** — §7.10 yellow band (70 °C) |
  | `red` | bearing/wheel condition band — red band (95 °C), and `FLAT_WHEEL` |
  | `crit` | non-band critical: `DERAIL`, `IMPACT`, `TAMPER`, `DOOR_UNAUTH`, `HANDBRAKE_MOVING`, `TILT`, `TANK_OVERTEMP`, `OVERLOAD`, `LOW_BATTERY` |
  | `warn` | non-band advisory: `SENSOR_FAULT`, `NODE_LOW_BATTERY` |

  > **Do not treat `yellow` as `red`.** A bearing at 70 °C and one at 95 °C are
  > different operational states — "watch this" versus "stop the train" — and
  > §7.16 escalation should differ accordingly. Earlier firmware published `red`
  > for both, which is why this distinction is new.

- `unit`: `C` (°C), `state` (1 open / 0 closed), `%`, `g`, `idx` (vibration
  index), or `cg` (centi-g, peak |acceleration| ×100) on a `TAMPER`. `thr` is 0
  where not applicable.
- `pos`: physical location of the sensor that raised it (bogie/axle/side).
  For chassis-level alarms (e.g. `IMPACT`, `LOW_BATTERY`) `node` is `null`.
- `st`: `set` when the condition starts, `clear` when it ends — one message per
  transition (not repeated every heartbeat). Pair them by `aid`.

## 5. Event (`mt:"event"`)

```json
"d": { "code":"TRAIN_START", "from":"stopped", "to":"running",
       "node":null, "trip":"trip-7", "info":{} }
```

State changes for the wagon as a whole: `TRAIN_START` / `TRAIN_STOP`, door/load
change events, etc. `trip` groups a run.

### 5.1 `node` — chassis-level vs node-level

`node` is `null` only when the condition belongs to the **wagon**, not to one
sensor. It carries the **sensor serial** for a node-level event:

| Event | `node` |
|---|---|
| `BOOT`, `TRAIN_START`, `TRAIN_STOP` | `null` — wagon-level |
| `CONN_RESTORED`, `SRC_SWITCH`, `CHARGE_START/STOP` | `null` — gateway-level |
| `GEOFENCE_ENTER` / `GEOFENCE_EXIT` | `null` — wagon position |
| **`DOOR_OPEN` / `DOOR_CLOSE`** | **`"NODE-DOOR-13"`** |
| **`BAND_CHANGE`** | **`"NODE-BRG-03"`** |
| **`LOAD_CHANGE`** | **`"NODE-LOAD-08"`** |
| **`HANDBRAKE_APPLIED` / `_RELEASED`** | **`"NODE-HB-12"`** |

> ⚠ **Was wrong in the firmware, now fixed.** Every event used to publish
> `"node":null`, which made `DOOR_OPEN` identical across all four door nodes —
> the cloud was told a door opened but never which one. Node-level events now
> carry the serial. A consumer that assumed `node` was always `null` needs
> updating.

## 6. Response (`mt:"resp"`)

Reply to a downlink command; correlate with the command's `cid`. Full command
list and payloads are in **Downlink_Command_Reference.md**.

```json
"d": { "cid":"i01","cmd":"set_interval","res":"ok","ec":null,"pl":{"applied":true} }
```

## 7. Units & conventions (so the reader never guesses)

| Type (`typ`) | `val` meaning | `unit` | `t` (secondary) |
|---|---|---|---|
| `btemp` (bearing) | temperature | `C` | vibration index |
| `load` | load state/percent | `state` | tilt |
| `hbrake` | 1 engaged / 0 released | `state` | — |
| `door` | 1 open / 0 closed | `state` | — |
| `ttemp` (tank) | temperature | `C` | — |

Notes: `ts` is UTC epoch seconds; `age` and `up` are seconds/hours respectively;
`spd` is km/h; `pos` is free text set per fitment (recommended
`B<bogie>-A<axle>-<L|R>` for bearings). Bearing vibration (`t`) is only valid
when `spd > 15`.

### 7.1 Bearing vibration reaches you faster than the node cadence suggests

A bearing node normally advertises every 30 s. When its vibration index crosses
the node's own hint limit it sets an internal `VIB_HIGH` flag and drops to **1 s**
advertising, so the gateway gets fresh data within about a second.

**That flag is not published** — it is a BLE-layer hint, not telemetry. The
`FLAT_WHEEL` alarm is still raised by the **gateway**, because the test is only
valid above 15 km/h and a bearing node has no accelerometer and no speed input:
it cannot tell rolling from shunting. What you see on MQTT is unchanged; it
simply arrives sooner.

Before this, the node escalated for its own over-temperature alarm but not for a
gateway-decided one, so a flat wheel could be reported up to 30× slower than an
over-temperature on the very same node (RDSO §7.3 asks for a *"near real time
alert"* once a threshold is exceeded).

The hint limit is settable per node — `set_threshold` with `"param":"vib_index"`
— and should be kept **at or below** the gateway's `FLAT_WHEEL_VIB_THRESH`, or
there is a band where the gateway would alarm but the node never speeds up.

## 8. Minimal reader recipe

1. Subscribe `smartwagon/v1/+/+/up/#`.
2. Parse JSON; key each record by `wgn` (wagon) and, for sensors, `pos` (physical
   location) — both are in the payload, no topic parsing needed.
3. De-duplicate/order on `(gw, seq)`.
4. For dashboards: group `hb.d.sens[]` by wagon, then by the bogie prefix of
   `pos`. For alerting: act on `alarm` where `st="set"`, resolve on `st="clear"`.

---
*Ref: RDSO WD-35-MISC-2024 · SmartWagon Telemetry Protocol Rev.1 (pv=1) · nRF54L15 (QFN52)*
