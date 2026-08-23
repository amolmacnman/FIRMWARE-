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
  "spd": 42,               // km/h
  "pwr": { "src":"lto","soc":78,"sol":"charging","bkp":"ok" },
  "d": { ...type-specific body... }
}
```

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
| `bat`  | Node battery %. |
| `age`  | Seconds since this node was last heard over BLE. |
| `miss` | `1` = this fitted node was **not heard** this cycle (treat as stale). |

To locate a hot bearing on wagon X: filter `wgn == X`, then read
`sens[].pos` — no id math. To roll up per bogie, group by the `B<n>` prefix of
`pos`.

## 4. Alarm (`mt:"alarm"`)

```json
"d": {
  "code":"HOT_BEARING", "sev":"red",
  "node":"NODE-BRG-06", "pos":"B2-A2-L",
  "val":97, "unit":"C", "thr":95,
  "st":"set", "aid":"al-4219", "spd":42
}
```

- `code`: `HOT_BEARING`, `FLAT_WHEEL`, `TILT`, `OVERLOAD`, `HANDBRAKE_MOVING`,
  `DOOR_UNAUTH`, `DOOR_TAMPER`, `TANK_OVERTEMP`, `TANK_TAMPER`, `IMPACT`,
  `SENSOR_FAULT`, …
  - `DOOR_UNAUTH` = door reported open while the gateway's motion state says the
    train is **moving** (open-while-stopped is heartbeat state, not an alarm).
  - `DOOR_TAMPER` / `TANK_TAMPER` = an **impact/shock** was detected on that node
    (raised regardless of motion). The two are distinguished by the node's
    `IMPACT` flag; a plain open-in-transit is `DOOR_UNAUTH`.
- `unit`: `C` (°C), `state` (1 open / 0 closed), or `cg` (centi-g, i.e. peak
  |acceleration| ×100) for the `*_TAMPER` impact alarms. `thr` is 0 where not
  applicable (door/impact).
- `pos`: physical location of the sensor that raised it (bogie/axle/side).
  For chassis-level alarms (e.g. `IMPACT`) `node` is `null`.
- `st`: `set` when the condition starts, `clear` when it ends — one message per
  transition (not repeated every heartbeat). Pair them by `aid`.
- `sev`: `red` (critical) / `warn` (advisory).

## 5. Event (`mt:"event"`)

```json
"d": { "code":"TRAIN_START", "from":"stopped", "to":"running",
       "node":null, "trip":"trip-7", "info":{} }
```

State changes for the wagon as a whole: `TRAIN_START` / `TRAIN_STOP`, door/load
change events, etc. `trip` groups a run.

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

## 8. Minimal reader recipe

1. Subscribe `smartwagon/v1/+/+/up/#`.
2. Parse JSON; key each record by `wgn` (wagon) and, for sensors, `pos` (physical
   location) — both are in the payload, no topic parsing needed.
3. De-duplicate/order on `(gw, seq)`.
4. For dashboards: group `hb.d.sens[]` by wagon, then by the bogie prefix of
   `pos`. For alerting: act on `alarm` where `st="set"`, resolve on `st="clear"`.

---
*Ref: RDSO WD-35-MISC-2024 · SmartWagon Telemetry Protocol Rev.1 (pv=1) · nRF54L15 (QFN52)*
