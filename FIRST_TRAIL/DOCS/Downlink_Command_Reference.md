# Smart Wagon — Downlink Command Reference (`dn/cmd`)

Server → gateway control channel for the SmartWagon Telemetry Protocol (Rev.1).
Controller: nRF54L15. This document is the contract the **server side** builds
against.

> **Every field below is verified against `handle_command()` in
> `GATEWAY/src/main.c`.** Where an earlier revision of this document disagreed
> with the firmware, the firmware wins and the difference is called out in a
> **⚠ Was documented wrong** note. Read those notes before writing a server
> integration — two of them (the response envelope and sub-node threshold
> units) will silently break a client that trusts the old text.

---

## 1. Topics

**There is exactly ONE downlink topic**, as defined by Protocol Rev.1 §7.1:

| Direction | Topic | QoS | Retain |
|---|---|---|---|
| Command (server → gateway) | `smartwagon/v1/{wgn}/{gw}/dn/cmd` | 1 | **false** |
| Response (gateway → server) | `smartwagon/v1/{wgn}/{gw}/up/resp` | 1 | false |

`{wgn}` = wagon number, `{gw}` = gateway id — both derived from the wagon
number, so a wagon's topic never changes.

> ⚠ **Was documented wrong.** A previous revision of this document described
> two extra broadcast topics, `smartwagon/v1/grp/{group}/dn/cmd` and
> `smartwagon/v1/all/dn/cmd`, and the firmware subscribed to them.
> **Both have been removed.** Neither appears in Protocol Rev.1, and both break
> its base pattern `smartwagon/v1/{wgn}/{gw}/{dir}/{kind}`: the cloud
> subscriptions the spec itself documents in §7.2 (`smartwagon/v1/+/+/up/#`)
> assume position 3 is a wagon number, so a literal `grp` there mis-parses for
> any subscriber built to the spec. `FLEET_GROUP` is gone from `app_config.h`
> with them. **Do not publish to those topics — nothing is listening.**

### 1.0 The `status` topic - link state

| Topic | QoS | Retain | Values |
|---|---|---|---|
| `smartwagon/v1/{wgn}/{gw}/status` | 1 | **true** | `online` / `offline` |

**The gateway publishes both values itself.** It is not inferred by the broker.

| Value | Published | When |
|---|---|---|
| `online` | by the gateway | retained, immediately after the broker session is established |
| `offline` | by the gateway | retained, immediately before it disconnects and powers the modem down |
| `offline` | by the broker (Last Will) | **backstop only** - fires when a session ends without a DISCONNECT, i.e. the gateway crashed or lost signal before it could publish |

The Last Will is configured (`AT+QMTCFG="will"` before `QMTCONN`) but is a
safety net, not the mechanism: it acts only in the cases where the gateway
physically cannot publish for itself. Under normal cycling the broker discards
it on the clean DISCONNECT, and the gateway's own `offline` is what you see.

> **Design the back office around this: `offline` is the NORMAL state.** The
> gateway powers its modem up for roughly 30 s per report and is deliberately
> off the rest of the time - about 99 % of it at the RDSO §7.4 ten-minute
> cadence. So this topic reads `offline` almost whenever anyone looks, and a
> healthy cycling gateway is indistinguishable from a dead one by this field
> alone.
>
> **Liveness must come from staleness on the heartbeat `ts`**: if no heartbeat
> has arrived in ~3 cadence periods, treat the wagon as down. `status` tells you
> whether the link is up at this instant; only the heartbeat clock tells you
> whether the gateway is alive.

Only `status` is retained. The `up/` topics are non-retained per Protocol
Rev.1 §7.1, so a late subscriber gets fresh telemetry rather than a stale
snapshot.

### 1.1 Group and bulk commands (RDSO §5.1.13)

§5.1.13 requires the FUOTA mechanism to accept *"individual, group wise or bulk
upgrade commands"*, and the gateway does. The **addressing** simply stays inside
the topic tree the railway specified:

- The command carries **`"scope"`** in its payload — `individual`, `group` or
  `bulk`.
- The **cloud performs the fan-out**: one publish per wagon, each on that
  wagon's own `dn/cmd` topic.

The cloud is the right place for this. It already knows the current rake
composition, and a rake is re-marshalled constantly — a group id stored on the
wagon would go stale at the next formation. Per-wagon publishing also gives
**delivery confirmation per wagon**, which a broadcast to a shared topic never
could.

Cost is one QoS 1 publish per wagon from a server. For a 58-wagon rake that is
58 publishes — negligible server-side.

### 1.2 Rules for every publish

- **Responses always go to the wagon's OWN `up/resp`.** After a fan-out, expect
  N responses arriving over hours as wagons wake. Correlate by `cid` **and**
  `gw`.
- **Never publish with `retain=true`.** The gateway subscribes to `dn/cmd` once
  per wake cycle, so the broker would re-deliver a retained command on **every**
  cycle, forever. A retained `time_sync` walks the clock backwards each cycle;
  a retained `reboot` is a boot loop; a retained `ota_start` re-downloads the
  image over cellular indefinitely.
- **Use QoS 1.** The broker's persistent session (`AT+QMTCFG="session",0,0`)
  queues the command while the gateway sleeps and delivers it on the next
  connect. A QoS 0 publish is only seen if the gateway happens to be connected
  at that instant — a window of a few seconds per cycle.

## 2. Timing — read this first (Class-A device)

The gateway is a sleeping, uplink-first (Class-A) device. **It only processes
queued commands when it wakes and connects** — right after one of its own
uplinks. A command may therefore sit queued at the broker from **seconds up to
the full stopped-wagon reporting interval** before it is executed and answered.

There is no server push path to wake a sleeping wagon; you wait for the next
scheduled wake. Design the server UI to treat responses as **asynchronous** and
correlate them by `cid`.

If a command never produces a response, **re-publish it**. QoS 1 queueing only
holds while the broker keeps the session; if the gateway's `QMTCONN` fails or
the session is dropped, the queue can be lost.

## 3. Common request envelope

Every command is a single JSON object:

```json
{ "mt": "cmd", "cid": "a1b2", "cmd": "<command>", "...": "command-specific fields" }
```

| Field | Type | Req | Notes |
|---|---|---|---|
| `mt` | string | yes | Always `"cmd"`. |
| `cid` | string (≤31) | yes | Correlation id, echoed in the response. Make it unique per outstanding command. |
| `cmd` | string | yes | One of the commands in §5. |
| `scope` | string | no | `individual` (default), `group`, `bulk`/`all`. See below. |

**`scope` declares the operator's intent**, since every command now arrives on
the same per-wagon topic (§1) and the gateway therefore cannot infer it.

It changes exactly **two** behaviours, both on `ota_start` (§5.10):

1. The download is **staggered** by a deterministic per-wagon offset, so a
   fan-out to hundreds of wagons does not hit the image server at once.
2. A wagon already running the named `ver` answers `ok` / `"current"` and does
   nothing — which is what makes a fan-out safe to repeat.

Every other command behaves identically however it was addressed, so `scope` is
optional and harmless elsewhere. Setting `scope:"group"` on a one-wagon command
merely staggers an update that did not need staggering.

**Whitespace is allowed** around colons — `{ "cid": "i01" }` and
`{"cid":"i01"}` both parse. Unknown fields are ignored.

## 4. Common response envelope

The gateway answers each command on `up/resp`:

```json
"d": { "cid":"i01", "cmd":"set_interval", "res":"ok", "ec":null,
       "pl":{"applied":true} }
```

| Field | Type | Notes |
|---|---|---|
| `cid` | string | Same `cid` as the request. |
| `cmd` | string | Echo of the command. |
| `res` | string | `"ok"` or `"err"`. |
| `ec` | string / null | Error code when `res="err"`, otherwise `null`. |
| `pl` | object | Command-specific payload. `{}` where there is none. |

> ⚠ **Was documented wrong.** This document previously specified `result`,
> `reason`, and command fields flattened into `d`. The firmware emits
> **`res`**, **`ec`**, and wraps the payload in **`pl`**. A parser looking for
> `result` finds nothing.

**Error codes** (`ec`): `UNSUPPORTED` (unknown command, or a supported command
refused by design), `BAD_PARAM` (missing/invalid/out-of-range field), `BUSY`
(an OTA is already running), `NOT_FOUND` (target node not yet heard from),
`FAILED` (execution error).

The response is wrapped in the standard uplink envelope (`mt`, `pv`, `seq`,
`gw`, `wgn`, `ts`, `loc`, `spd`, `pwr`) — see `MQTT_Uplink_Schema.md`. Only the
`d` object is shown per command below.

---

## 5. Gateway commands

### 5.1 `get_config` — read the whole active configuration

No parameters. The cheapest, safest first test: it changes nothing.

```json
{ "mt": "cmd", "cid": "c01", "cmd": "get_config" }
```

**Response**

```json
"d": { "cid":"c01", "cmd":"get_config", "res":"ok", "ec":null,
       "pl": { "run_s":30, "stop_s":60, "impact_mg":4000,
               "gnss_enable":1, "gnss_constel":3, "gnss_timeout_s":90,
               "nodes":19, "fitment":524287, "wagon":"31054312345",
               "batt_full_mv":5400, "batt_empty_mv":4000,
               "fw":"1.0.0" } }
```

`gnss_constel`: 0 = auto, 1 = navic, 2 = gps, 3 = navic_gps.
An `args{section}` field is accepted and ignored — the whole block fits in one
response, so splitting it would only add a round trip on a Class-A link.

---

### 5.2 `get_status` — force an immediate report

No parameters. Publishes an **out-of-schedule heartbeat** on `up/hb`, then
answers.

```json
{ "mt": "cmd", "cid": "s01", "cmd": "get_status" }
```

**Response**

```json
"d": { "cid":"s01", "cmd":"get_status", "res":"ok", "ec":null,
       "pl":{"applied":true} }
```

> ⚠ **Was documented wrong.** This previously showed a rich snapshot (`fw`,
> `uptime_s`, `batt`, `lat`, `lon`, `nodes_down`, …) in the response. The
> firmware returns only `{"applied":true}` — **the data arrives in the
> heartbeat it triggers**, not in the response. Read `up/hb` for the state.

---

### 5.3 `pull_data` — on-demand sensor snapshot

No parameters. Also answered by forcing a heartbeat, which already carries the
full `sens[]` array.

```json
{ "mt": "cmd", "cid": "d01", "cmd": "pull_data" }
```

**Response**

```json
"d": { "cid":"d01", "cmd":"pull_data", "res":"ok", "ec":null,
       "pl":{"nodes":19,"via":"hb"} }
```

---

### 5.4 `get_history` — how much is buffered

```json
{ "mt": "cmd", "cid": "h01", "cmd": "get_history", "from_seq": 0 }
```

| Field | Type | Req | Notes |
|---|---|---|---|
| `from_seq` | int | no | Echoed back; the reply is a count, not a range extract. |

**Response**

```json
"d": { "cid":"h01", "cmd":"get_history", "res":"ok", "ec":null,
       "pl": { "from_seq":0, "count":0, "mode":"push",
               "info":"records replay on reconnect with original seq/ts" } }
```

Only the **count** is returned, not the records: each buffered record is a full
envelope up to ~640 B and the response would exceed the modem's publish buffer.
If `count > 0` the gateway immediately starts a `telem_flush_backlog()`, which
republishes each record on its own topic with its **original** `seq` and `ts`,
so `(gw, seq)` de-duplication still works.

`count` is always **0 on a board with no external NOR fitted** (e.g. the
nRF54L15 DK) — there is no archive there, and anything that fails to publish is
lost rather than buffered.

---

### 5.5 `set_interval` — change reporting cadence

```json
{ "mt": "cmd", "cid": "i01", "cmd": "set_interval", "moving_s": 600, "idle_s": 43200 }
```

| Field | Type | Req | Valid range | Notes |
|---|---|---|---|---|
| `moving_s` | int (s) | no* | **30 – 86400** | Heartbeat period while moving. |
| `idle_s` | int (s) | no* | **60 – 604800** | Heartbeat period while stopped. |

\* Send either or both. **If neither field is present or in range, the command
returns `err/BAD_PARAM` and nothing is changed** — there is no partial apply.

**Response**

```json
"d": { "cid":"i01", "cmd":"set_interval", "res":"ok", "ec":null,
       "pl":{"applied":true} }
```

The new cadence takes effect at the next `arm_schedule()` (i.e. after the
current report) and is persisted to FRAM.

**Do not expect a cadence shorter than one cycle to be honoured.** A cycle
spends up to `gnss_timeout_s` (default 90 s) in `acquire_fix()` before the
modem is even powered, so a 30 s cadence on a board with no GNSS fix simply
re-arms while the previous report is still running. Lower `fix_timeout_s` or
disable GNSS (§5.6) if you need a fast bench loop.

---

### 5.6 `set_gnss` — change GNSS behaviour

```json
{ "mt": "cmd", "cid": "g01", "cmd": "set_gnss", "enable": true,
  "fix_timeout_s": 90, "constellation": "navic_gps" }
```

| Field | Type | Req | Valid range | Notes |
|---|---|---|---|---|
| `enable` | bool | no | — | `false` = stop taking fixes and report last-known position. |
| `fix_timeout_s` | int (s) | no | **10 – 300** | Max wait per report. Out-of-range values are ignored silently. |
| `constellation` | string | no | `navic`, `gps`, `navic_gps`, `auto` | Anything else maps to `auto`. |

Pushed to the LC29H immediately (the GNSS rail is powered at this point in the
cycle) and persisted via PAIR513 plus FRAM.

**Response**

```json
"d": { "cid":"g01", "cmd":"set_gnss", "res":"ok", "ec":null,
       "pl":{"applied":true} }
```

`set_gnss` **always** returns `ok`, even if every field was absent or
out-of-range — unlike `set_interval`, it does not validate into `BAD_PARAM`.
Confirm with `get_config`.

---

### 5.7 `time_sync` — set the clock

```json
{ "mt": "cmd", "cid": "t01", "cmd": "time_sync", "epoch": 1786060800 }
```

| Field | Type | Req | Notes |
|---|---|---|---|
| `epoch` | int | yes | UTC seconds since 1970-01-01. Must be `> 0`, else `BAD_PARAM`. |

**Response**

```json
"d": { "cid":"t01", "cmd":"time_sync", "res":"ok", "ec":null,
       "pl":{"applied":true} }
```

The clock is **software only** — it free-runs off `k_uptime` between GNSS
fixes, and **every GNSS fix silently re-disciplines it**. It is held in RAM, so
a reset returns `ts` to `0` until the next fix or `time_sync`. A hardware RTC
on a backup domain is a future addition.

`ts:0` in any uplink means "clock never set" and must not be read as 1970.

---

### 5.8 `set_threshold` — change an alarm limit

Two distinct behaviours depending on whether `node` is present.

#### 5.8a Gateway-held threshold

Only **`impact_g`** is held by the gateway.

```json
{ "mt": "cmd", "cid": "x01", "cmd": "set_threshold",
  "param": "impact_g", "value": 4.0, "hyst": 0.5 }
```

| Field | Type | Req | Valid range |
|---|---|---|---|
| `param` | string | yes | Must be `impact_g`. |
| `value` | number (g) | yes | **> 0.5 and ≤ 16.0** |
| `hyst` | number | no | Accepted; not applied to `impact_g`. |

Stored as `impact_mg` (value × 1000) and persisted to FRAM.

**Response** — `pl:{"applied":true}`, or `err/BAD_PARAM` if out of range.

#### 5.8b Sub-node threshold — see §6

```json
{ "mt": "cmd", "cid": "n01", "cmd": "set_threshold",
  "node": 17, "value": 550, "hyst": 50 }
```

---

### 5.9 `reboot` — cold restart

The gateway answers **first**, waits 500 ms so the response is delivered, then
resets.

```json
{ "mt": "cmd", "cid": "r01", "cmd": "reboot", "mode": "cold" }
```

| Field | Type | Req | Notes |
|---|---|---|---|
| `mode` | string | no | Accepted and ignored; the reset is always cold. |

**Response** — `pl:{}`.

Note that a reset clears the software clock (§5.7) and, on a board without
FRAM, all runtime configuration reverts to the compile-time defaults.

---

### 5.10 `ota_start` — start a firmware update

Downloads a **signed** image for the **gateway only** and applies it via
MCUboot with signature, version and downgrade checks plus auto-revert on an
unconfirmed boot. MQTT signals the URL; HTTP transfers the bytes. The image is
staged in MCUboot's secondary slot in internal flash, so an update can never
erase buffered telemetry on the external NOR.

```json
{ "mt": "cmd", "cid": "o01", "cmd": "ota_start",
  "url": "http://ota.example.com/gw/sw_gateway_1.3.0.bin",
  "ver": "1.3.0",
  "size": 328904 }
```

| Field | Type | Req | Notes |
|---|---|---|---|
| `url` | string (≤159) | yes | Location of the signed image. |
| `ver` | string (≤23) | yes | Target version. |
| `size` | int (bytes) | yes | Image size, for progress and a sanity check. |
| `node` | int | — | **Presence alone causes `err/UNSUPPORTED`.** |

> ⚠ **Was documented wrong.** `target` and `sha256` were documented as request
> fields. **Neither is read by the firmware.** There is no `target` dispatch —
> the presence of `node` is what rejects a sub-node update — and integrity
> comes from the MCUboot signature, not from a `sha256` field. Sending them is
> harmless but they do nothing.

**Response** (accepted) — the OTA state block:

```json
"d": { "cid":"o01", "cmd":"ota_start", "res":"ok", "ec":null,
       "pl":{"state":"downloading","pct":0,"ver":"1.3.0"} }
```

**Response** (already running) — `res:"err"`, `ec:"BUSY"`.
**Response** (`node` present) — `res:"err"`, `ec:"UNSUPPORTED"`,
`pl:{"info":"node OTA not supported"}`.

#### Group and bulk updates (FUOTA — RDSO §5.1.13)

Publish the **same** `ota_start` to the group or bulk topic and add `scope`:

```json
{ "mt": "cmd", "cid": "o10", "cmd": "ota_start", "scope": "group",
  "url": "http://ota.example.com/gw/sw_gateway_1.3.0.bin",
  "ver": "1.3.0", "size": 328904 }
```

The cloud publishes this once **per wagon**, each on that wagon.s own
`smartwagon/v1/{wgn}/{gw}/dn/cmd` (SS1.1). There is no broadcast topic - the
`scope` field is what tells each gateway it is part of a fan-out.

#### Restricting a fleet update to one fitment

A fleet-scoped `ota_start` may carry **`fitment`** — the mask (SS5.13) the image
was built for. A gateway whose fitment differs refuses **before downloading**:

```json
{ "mt": "cmd", "cid": "o11", "cmd": "ota_start", "scope": "group",
  "fitment": 155903,
  "url": "http://ota.example.com/gw/gw_fit155903_1.4.0.bin",
  "ver": "1.4.0", "size": 295244 }
```

```json
"d": { "cid":"o11", "cmd":"ota_start", "res":"err", "ec":"FITMENT_MISMATCH",
       "pl":{ "fitment":131072, "image_fitment":155903 } }
```

The field is **optional** — an operator addressing one wagon deliberately can
omit it, and individual updates are unaffected.

It exists because identity left the image but the **roster did not**: a gateway
running another fitment's build expects nodes that were never installed, reports
them missing and raises `SENSOR_FAULT` continuously while ignoring nodes it does
have. Refusing costs one MQTT round trip; accepting costs ~290 KB of metered
cellular to install a build that cannot work.

`get_status` reports each gateway's `fitment`, so the back office can group
wagons without maintaining its own register.

Three behaviours make this safe at fleet scale:

**1. Responses come back staggered, in two parts.** A fleet-scoped update
answers **immediately** with `state:"scheduled"`, then waits out this wagon's
slot before downloading:

```json
"d": { "cid":"o10", "cmd":"ota_start", "res":"ok", "ec":null,
       "pl":{"state":"scheduled"} }
```

Only a **failure** produces a second response. A successful fleet update stays
silent after `scheduled` — confirm it completed by reading `fw` in the wagon's
next heartbeat, or with `ota_status`.

**2. Downloads are spread deterministically.** Each gateway delays by
`CRC16(wagon_number) mod OTA_FLEET_STAGGER_MS` (default window **10 minutes**).
2500 wagons hitting the firmware host in the same second would be a
self-inflicted DDoS and a burst the cellular network will shed. The offset is
derived from the wagon number rather than randomised, so it is **fixed and
reproducible** — you can compute exactly when any given wagon will start, and
two wagons never collide by chance. An **individual** `ota_start` is never
staggered.

**3. Repeats are free.** A roll-out is not atomic — wagons are asleep or out of
coverage — so you will re-publish the same command over days. A gateway already
running `ver` answers without downloading anything:

```json
"d": { "cid":"o10", "cmd":"ota_start", "res":"ok", "ec":null,
       "pl":{"state":"current"} }
```

`res:"ok"`, not an error: mark that wagon **done** rather than retrying it.

> **Never publish a group or bulk command with `retain=true`.** It would be
> re-delivered to every gateway on every reconnect — the whole fleet
> re-downloading forever. The version guard turns that into a no-op after the
> first pass, but it is a backstop, not a licence to retain.

**Roll-out recipe:** publish to the group topic → collect `scheduled`
responses to learn which wagons are awake → re-publish daily → wagons answering
`current` are done → wagons never answering need investigation.

---

### 5.11 `ota_status` — query an update in progress

No parameters.

```json
{ "mt": "cmd", "cid": "o02", "cmd": "ota_status" }
```

**Response**

```json
"d": { "cid":"o02", "cmd":"ota_status", "res":"ok", "ec":null,
       "pl":{"state":"idle","pct":0,"ver":"1.0.0"} }
```

`state` ∈ `idle`, `downloading`, `verifying`, `ready`, `applying`, `error`.

---

### 5.12 Provisioning — what is settable at runtime

Identity used to be compiled in, so this section refused every provisioning
command. That is no longer true, and the change is what makes one firmware
image serve a whole fleet.

| Value | Where it lives | Set by |
|---|---|---|
| Gateway wagon number | gateway on-chip NVS | [`set_wagon`](#515-set_wagon--provision-the-wagon-number) |
| Gateway fitment (which nodes exist) | gateway on-chip NVS | [`set_threshold param:"fitment"`](#513-set_threshold-paramfitment--which-nodes-this-wagon-has) or [`learn_fitment`](#514-learn_fitment--adopt-what-is-actually-heard) |
| Sub-node id | sub-node NVS | `set_threshold param:"node_id"` (§6.11) |
| Sub-node wagon number | sub-node NVS | **build seed only** — SWD reflash to change |
| Cadence, thresholds | gateway FRAM / node NVS | §5.5, §5.8, §6 |

The compile-time values in `app_config.h` and `nodes.c` are **factory seeds**,
used only when storage holds nothing. Once provisioned, storage wins and
survives every firmware update — which is precisely what lets a fleet-wide
image carry no identity of its own.

**The sub-node wagon number is deliberately not settable over the air.** The
downlink payload is six bytes (`cmd`, `rsvd`, and two 16-bit operands); an
11-digit wagon number does not fit, and splitting it across frames would need
chunk indices, reassembly and a commit step — a protocol of its own, to set a
value once in a node's life. It is not needed either: the build seed writes NVS
on first boot and later updates leave it alone. Moving a node to a different
wagon means a reflash, which is already a workshop operation.

Any command name not in §5 also returns `err/UNSUPPORTED` with `pl:{}`.

### 5.13 `set_threshold param:"fitment"` — which nodes this wagon has

Tells the gateway which sensor nodes are fitted, as a **19-bit mask**: bit *N*
set means node id *N* is present.

```json
{ "mt":"cmd", "cid":"f1", "cmd":"set_threshold",
  "param":"fitment", "value":155903 }
```

```json
"d": { "cid":"f1", "cmd":"set_threshold", "res":"ok",
       "pl":{ "fitment":155903, "image":131072, "nodes":11 } }
```

| ids | sensor | | fitment | hex | decimal |
|---|---|---|---|---|---|
| 0–7 | bearings | | all 19 nodes | `0x7FFFF` | 524287 |
| 8–11 | load / tilt | | 8 bearings only | `0x000FF` | 255 |
| 12 | handbrake | | 1 tank temperature | `0x20000` | 131072 |
| 13–16 | doors | | 2 tank temperature | `0x60000` | 393216 |
| 17–18 | tank temperature | | 8 bearing · 2 door (B1) · 1 tank | `0x260FF` | 155903 |

**Which nodes, not how many.** "8 bearings, 2 doors, 1 tank" is three different
fitments depending on which two doors: `0x260FF`, `0x2A0FF`, `0x380FF` — all
eleven nodes. A wagon given the wrong one reports the doors it has as missing
and the ones it lacks as present, permanently.

**Send `0` to follow the image again.** The stored value is an *override*, not
a copy, so a firmware update carrying a new default still reaches every wagon
that never overrode, while a retro-fitted wagon keeps its own answer through
that same update.

Takes effect immediately — no reboot. The mask is also what a group `ota_start`
is addressed to (§5.10), and `get_status` reports it so the back office never
has to keep its own fitment register in step.

---

### 5.14 `learn_fitment` — adopt what is actually heard

```json
{ "mt":"cmd", "cid":"f2", "cmd":"learn_fitment" }
```

```json
"d": { "cid":"f2", "cmd":"learn_fitment", "res":"ok",
       "pl":{ "fitment":393216, "nodes":2, "image":131072 } }
```

Sweeps node ids 0–18, adopts every one it can hear, and stores the result.
Intended for commissioning: computing a bitmask by hand for hundreds of wagons
invites exactly the "which two doors" mistake above, and the gateway already
knows — it has been listening to those nodes since power-up.

**Run it with every node powered and in range, then check `nodes` against the
work order.** A node asleep or out of range during the sweep is silently absent
from the result, and that count is the only thing that catches it.

> **One-shot, never continuous.** Keeping the roster automatically in sync with
> what is heard would mean a node that *died* was quietly forgotten instead of
> reported missing — the exact fault `SENSOR_FAULT` exists to catch. The roster
> is a statement of what **should** be there, so it can only be set on purpose.

Returns `err/NOT_FOUND` if nothing is heard at all.

---

### 5.15 `set_wagon` — provision the wagon number

```json
{ "mt":"cmd", "cid":"w1", "cmd":"set_wagon", "wagon":"31054312345" }
```

```json
"d": { "cid":"w1", "cmd":"set_wagon", "res":"ok",
       "pl":{ "wagon":"31054312345", "grp":17991, "rebooting":true } }
```

Stores the wagon number in the gateway's on-chip NVS and **reboots**. The
reboot is not optional: the MQTT client id, the topic prefix, the BLE isolation
group (CRC16) and the per-wagon AES-CCM key (HKDF) are all derived from it once
at startup, and rebuilding them in place would mean tearing down the broker
session and the BLE filter mid-cycle.

> ### ⚠ One-way over the air
>
> After the reboot the gateway answers on the **new** wagon's topic and is deaf
> to the old one. A command sent with the wrong number does not fail — it moves
> the gateway somewhere you may not be listening, and recovery means publishing
> to the new topic, which requires knowing what was sent. The response above
> carries the new number for exactly that reason: **it is the last thing the old
> topic ever hears.**

1–15 characters; anything else returns `err/BAD_PARAM`.

Storage is on-chip NVS rather than the FRAM that holds the rest of the config.
The FRAM is an external part behind a switched rail; a gateway that cannot read
its wagon number cannot decrypt its own nodes or publish to its own topic, so
identity is the last thing it should lose, not the first.

---

### 5.15b `set_ble_tx` — gateway BLE output power

```json
{ "mt":"cmd", "cid":"c60", "cmd":"set_ble_tx", "dbm": 8 }
```

Governs the gateway's **transmit** side only — the connection it opens to a node
during that node's config window, carrying thresholds, calibration and OTA
chunks. Applied to each connection as it comes up; no reboot.

**It does NOT change the RSSI you see for node adverts.** That number is the
gateway measuring the *node's* transmitter — use `set_threshold param:"ble_tx_dbm"`
(§6.15) for that.

| Guard | Behaviour |
|---|---|
| Range | **−40 … +8 dBm**. +8 is the nRF54L15 ceiling. Out of range is refused, not clamped — the controller would clamp silently, and an operator asking for +20 should be told it is impossible. |
| Persistence | stored with the rest of the config. |
| Rejection | `INVALID_THRESHOLD`, with the value still in force. |

+8 dBm is affordable here in a way it is not on a node: the gateway runs from a
solar-fed rechargeable LTO pack with no six-year primary-cell budget.

---

### 5.16 `set_batt` — calibrate the gateway pack's state-of-charge map

Maps LTO pack voltage to the `soc` percentage every uplink carries.
`full_mv` → 100 %, `empty_mv` → 0 %, linear between them.

```json
{ "mt":"cmd", "cid":"c40", "cmd":"set_batt",
  "full_mv": 5400, "empty_mv": 4000 }
```

**Response**

```json
"d": { "cid":"c40", "cmd":"set_batt", "res":"ok", "ec":null,
       "pl": { "applied":true, "full_mv":5400, "empty_mv":4000,
               "vbat_mv":4812, "soc":58 } }
```

The response echoes the **live pack voltage and the percentage it now maps to**,
so one command both sets the map and shows what it did to the current reading.

| Guard | Behaviour |
|---|---|
| Range | both **3000 … 5800 mV**. The pack is `LTO2S4P` (schematic sheet 2) — two lithium-titanate cells in series — so this is 1.50 … 2.90 V/cell, the chemistry's physical limits. |
| Separation | `full_mv − empty_mv` ≥ **500 mV**. Closer than that and one millivolt of ADC noise moves the reported percentage by whole digits; equal ends would divide by zero. |
| Persistence | stored with the rest of the config; survives reboot and OTA. |
| Effect | immediate, **no reboot** — nothing else is derived from these two values, so the next charger read uses the new map. |
| Rejection | `INVALID_THRESHOLD`, with the accepted range **and the values still in force** in the payload. |

**These are pack properties, not firmware properties.** The right numbers depend
on what the BQ25798 actually terminates at and where the power mux hands over to
the Li-SOCl2 backup — neither knowable until a real pack has been charged and
run down. The compile-time seeds (5400 / 4000) are an estimate from the cell
chemistry.

**How to measure them:**

1. Charge until the charger reports `chg:"done"`; read `lto.v` → that is `full_mv`.
2. Run down until `src` switches to `"bkp"`; read `lto.v` → that is `empty_mv`.
3. Send both with this command.

> **Why `full_mv` is seeded *below* the 2.8 V/cell charge limit.** 100 % has to
> be reachable. If the charger terminates below `full_mv`, the gateway never
> reports a full pack and every wagon looks like it has a failing solar panel.
> Erring low costs a little resolution at the top; erring high manufactures a
> fleet-wide false fault.

> **Why `empty_mv` is seeded *above* the 1.8 V/cell floor.** The pack never gets
> that low — the power mux hands over to the Li-SOCl2 backup at ≈4.1 V. 0 %
> should mean "the LTO has stopped being useful", not "the cells are about to be
> damaged". It also orders the alarms correctly: `SRC_SWITCH` (15 %) lands at
> ≈4210 mV and `LOW_BATTERY` (10 %) at ≈4140 mV, so the warning fires *before*
> the handover.

**This does not reach sub-nodes.** Their cell is a different chemistry with a
flat discharge curve and no voltage→percent map at all — see §6.13 and §6.14.

---

---

## 6. Sub-node commands

### 6.0 `node_window` — make nodes reachable quickly

Sub-nodes are connectable for only **4 s in every 600 s**, so a queued command
waits **~5 minutes on average**. That is the right trade for a fit-and-forget
node on a six-year cell, but it is painful when commissioning a rake. This puts
the targeted nodes into a temporary **fast window** — connectable every ~5 s —
after which they revert **by themselves**.

```json
{ "mt":"cmd", "cid":"w01", "cmd":"node_window", "all":true, "seconds":600 }
```

| Field | Type | Req | Notes |
|---|---|---|---|
| `seconds` | int | yes | How long to stay fast. **0 – 3600**; 0 cancels. |
| `node` / `type` / `all` | — | yes | Same selector as `set_threshold` (§6.1). |

**Response**

```json
"d": { "cid":"w01", "cmd":"node_window", "res":"ok", "ec":null,
       "pl":{"applied":false,"state":"queued","nodes":8,"seconds":600} }
```

Like every sub-node command it is **queued**, so the *first* one still waits for
a normal window. Once it lands, everything after it is near-instant. The usual
commissioning pattern is therefore: send `node_window` once, wait for the first
window, then push thresholds back to back.

**Power.** Fast mode is ~6.4 mA on the node (4 s connectable in every 5 s)
against 53 µA normally. Ten minutes costs **~1.1 mAh — about 0.01 %** of a
10,000 mAh cell; the 1-hour cap costs ~6.4 mAh, under 0.1 %. A node never put
into fast mode draws exactly what it drew before, so the six-year budget is
untouched.

**It cannot strand a node.** The revert is driven by a deadline on the node, not
by the gateway sending anything — an abandoned campaign, a gateway that never
reconnects, or a lost cancel all end the same way. It is also deliberately
**not persisted**, so a reset returns the node to normal duty.

`seconds` above 3600 returns `BAD_PARAM`. Omitting the node selector does too —
there is no implicit "everything".



Two commands reach a sub-node: **`node_window`** (§6.0, make it reachable
quickly) and **`set_threshold`** (below, change a limit). Everything in §5 is
gateway-scoped.

Both use the same node selector — `node` / `type` / `all` — and both answer
`"state":"queued"` rather than `applied`, because a node is only reachable
during its own connectable window.

```json
{ "mt": "cmd", "cid": "n01", "cmd": "set_threshold",
  "param": "primary", "node": 17, "value": 550, "hyst": 50 }
```

| Field | Type | Req | Notes |
|---|---|---|---|
| `param` | string | no | **Which sensor.** Default `primary`. See §6.1. |
| `node` | int | yes* | Sub-node id, **0 – 19**. Outside that → `BAD_PARAM`. |
| `all` | bool | yes* | `true` = every node heard on this wagon. |
| `type` | int | yes* | `sw_node_type` — every node of that type. |
| `value` | number | yes | New threshold **in the node's own raw units** — §6.2. |
| `hyst` | number | no | Release hysteresis, same raw units. Defaults to 0. |

\* Supply exactly one of `node`, `all`, or `type`. `all` and `type` together
behave as `type` — the type narrows the set.

> ⚠ **Was documented wrong.** A previous revision said *"`param` is ignored on
> this branch"*, because each node then had only one threshold. **Nodes now
> hold three**, and `param` selects between them. A command sent without
> `param` still targets the primary sensor, so old callers keep working — but
> `param` is no longer ignored.

### 6.1 The three axes compose

A `set_threshold` names **which sensor**, on **which nodes**, of **which
wagons** — and the three are independent:

| Axis | Field | Values |
|---|---|---|
| **Sensor** | `param` | `primary` · `impact_g` · `vib_index` |
| **Nodes** | `node` / `type` / `all` | one node · one node type · all heard |
| **Wagons** | `scope` + cloud fan-out (§1.1) | one · group · bulk |

**`param` values**

| `param` | Sets | Units | Node types that accept it |
|---|---|---|---|
| omitted, or `primary` | the node's own alarm threshold | node's raw units | **all** |
| `bearing_temp_c`, `tank_temp_c`, `temp_c` | alias for `primary` on temperature nodes | °C ×10 | bearing, tank |
| `impact_g` | BMA400 tamper limit | **milli-g** (2000 = 2.0 g) | **door, tank only** |
| `vib_index` | vibration hint limit | raw index | **bearing only** |

Anything else → `BAD_PARAM` with `{"info":"unknown param"}`.

**A node that lacks the sensor refuses the command.** That is what makes an
`"all": true` fan-out safe — a bearing node rejects `impact_g`, a door node
rejects `vib_index`, and neither stores a value it could never act on. The
`nodes` count in the response is how many were *queued*, not how many accepted;
check the node's next uplink to confirm.

**`impact_g` means two different things**, and the node selector disambiguates:

- **without** `node`/`type`/`all` → the **gateway's own** BMA400 (§5.7),
  applied instantly, value in **g** (0.5 – 16.0)
- **with** a node selector → the **sub-nodes'** BMA400, queued, value in
  **milli-g**

### 6.2 `vib_index` does not raise an alarm

Setting `vib_index` does **not** create a node-side alarm. The `FLAT_WHEEL`
decision stays on the gateway, which alone knows the wagon speed — the test is
only valid above 15 km/h, and a bearing node has no accelerometer and no speed
input, so it cannot tell rolling from shunting.

What the node threshold controls is **latency**. Crossing it makes the node
raise `SW_FLAG_VIB_HIGH` and drop its advert period from 30 s to **1 s**, so
the gateway gets fresh vibration data within about a second instead of waiting
up to a full advert period. Before this, a node escalated for its *own*
over-temperature alarm but not for a gateway-decided one, so a flat wheel was
reported up to 30× slower than an over-temperature on the same node — RDSO §7.3
asks for a *"near real time alert"* once a threshold is exceeded.

**Keep `vib_index` at or below the gateway's `FLAT_WHEEL_VIB_THRESH` (800).** A
node limit *above* the gateway's leaves a band where the gateway would alarm but
the node never escalates, putting the latency straight back.

### 6.3 Units are RAW — this is the easiest thing to get wrong

Nothing scales anywhere in the chain: the value is cast to `int16_t`, sealed
into the BLE frame, and written verbatim by the node's `sn_cfg_set()`. Send
what the node stores.

| Node type | ids | `value` unit | Firmware default | For 70 °C send |
|---|---|---|---|---|
| bearing (`btemp`) | 0–7 | **°C × 10** | 850 = 85.0 °C | `700` |
| load / tilt | 8–11 | percent | 95 = 95 % | — (`90` = 90 %) |
| handbrake | 12 | 1 / 0 | 1 | — |
| door | 13–16 | 1 / 0 | 1 | — |
| tank (`ttemp`) | 17–18 | **°C × 10** | 600 = 60.0 °C | `700` |

`hyst` uses the same scale: `50` = 5.0 °C on a temperature node, 5 % on a load
node.

> ⚠ **Was documented wrong.** §5.1 of the previous revision listed
> `param: "bearing_temp_c"` with `value` in degrees. Sending `"value": 70`
> expecting 70 °C actually sets **7.0 °C**, which alarms permanently. Always
> send the ×10 value for temperature nodes.

### 6.4 Values ARE validated — twice

This section previously warned that nothing in the chain range-checked a
threshold, and that the server was the only thing between a typo and a disabled
hot-axle detector. That was true and it is now fixed.

**A threshold is checked at two points:**

1. **On the gateway, before the frame is sealed.** This is the only place a
   refusal can still become an MQTT error you actually see — once the frame is
   queued, the response has already gone out as `queued`.
2. **On the node, on receipt.** A GATT error, as a backstop for anything that
   reaches it another way.

Rejected values are **not applied and not persisted**. The response names the
reason:

```json
{ "mt":"res", "cid":"t9", "cmd":"set_threshold", "res":"err",
  "ec":"INVALID_THRESHOLD",
  "pl":{ "applied":false, "state":"queued", "nodes":0, "out_of_range":2 } }
```

`out_of_range` counts nodes that refused the **value**. It is reported even on
success, because a mixed-type scope where some nodes silently dropped out is
exactly the case that would otherwise go unnoticed:

```json
"pl": { "nodes":8, "out_of_range":4 }     // 8 bearings took it, 4 doors could not
```

**`INVALID_THRESHOLD` and `NOT_FOUND` mean different things.** The first says
the node is present and awake and cannot reach the number you sent; the second
says nothing matched. Confusing them sends an engineer to check antennas for a
units mistake.

#### What is checked

| Parameter | Enforced range | Gateway | Node |
|---|---|---|---|
| `primary` — bearing | −400 … 1500 (°C×10) | ✅ | ✅ |
| `primary` — tank/cargo | −500 … 1500 (°C×10) | ✅ | ✅ |
| `primary` — load/tilt | 0 … 100 (%) | ✅ | ✅ |
| `primary` — door, handbrake | 0 or 1, `hyst` must be 0 | ✅ | ✅ |
| `hyst` (analogue types) | ≥ 0, and `value − hyst` ≥ sensor floor | ✅ | ✅ |
| `impact_g` | 500 … 16000 mg | ✅ | ✅ |
| `vib_index` | ≥ 0 only | ✅ | ✅ |
| `cadence` | 1 … 3600 s, and `hyst ≤ value` | ✅ | ✅ |
| `node_id` | 0 … 19, **explicit `node` scope only** | ✅ | ✅ |

Two limits are worth understanding rather than memorising.

**`value − hyst` must stay above the sensor floor.** A release point below the
floor gives an alarm that can set but never clear, which looks like a stuck
sensor and hides the real reading behind a latched alarm.

**`vib_index` is sign-checked only.** The index is a raw, board-specific scale
with no calibrated ceiling, so any upper bound would be invented rather than
derived. A negative limit is the one unambiguously wrong value: it escalates on
every sample and holds the node at 1 s advertising until the cell is flat.

#### What is still the caller's job

Validation rejects the **impossible**, not the **unwise**. A bearing threshold
of 40 °C is inside the sensor range and will be applied; whether it is a
sensible operating point is a policy question the firmware does not answer. The
recommended bands in §6.5 remain server-side guidance.

### 6.5 Limits per node type

**These ranges are now enforced** — at the gateway before the frame is sealed,
and again at the node (§6.4). A value outside them is refused, not applied, and
the response says `INVALID_THRESHOLD`.

Earlier revisions of this document said `node` 0–19 was the only check and that
everything below was server-side guidance. That is no longer true, and the
distinction that replaced it matters: firmware rejects the **physically
impossible**, while the **Recommended** column below remains advice. A bearing
threshold of 40.0 °C is inside the sensor range and will be applied — whether it
is a sensible operating point is a policy question the firmware does not
answer.

| Node type | `type` | Unit | Default | Sensor range | **Recommended** |
|---|---|---|---|---|---|
| Bearing | 1 | °C ×10 | 850 (85.0 °C) | — | **400 – 1500** |
| Load / tilt | 2 | percent | 95 | 0 – 100 | **50 – 100** |
| Handbrake | **3** | 1 / 0 | 1 | 0 or 1 | **0 or 1** |
| Door | 5 | 1 / 0 | 1 | 0 or 1 | **0 or 1** |
| Tank / cargo | **4** | °C ×10 | 600 (60.0 °C) | −500 – 1500 | **−400 – 1250** |

- **Bearing:** RDSO §7.10 condition bands are yellow **70 °C** (700) and red
  **95 °C** (950). A threshold below the yellow band makes the band reporting
  and the alarm disagree.
- **Tank:** the cargo front end (thermocouple or PT100) covers −50…+150 °C;
  a threshold outside that can never be reached, so the alarm is dead.
- **Load:** values outside 0–100 are meaningless — the node reports a percent.
- **Binary types:** door and handbrake compare against 1 or 0. Any other value
  is undefined behaviour in the node's comparison.

**Always send `hyst`.** It defaults to **0**, which means no release
hysteresis: a reading sitting on the threshold will set and clear the alarm
repeatedly, and every transition is a QoS 1 publish over cellular. `50` (5.0 °C)
is the firmware default for temperature nodes, `5` for load.

### 6.6 Updating many nodes at once

**All eight bearings on the wagon to 80.0 °C** — the common case, and the safe
way to use this:

```json
{ "mt": "cmd", "cid": "n02", "cmd": "set_threshold",
  "type": 1, "value": 800, "hyst": 50 }
```

**Every node heard on the wagon:**

```json
{ "mt": "cmd", "cid": "n03", "cmd": "set_threshold",
  "all": true, "value": 1, "hyst": 1 }
```

> **Prefer `type` over `all`.** One value across every type is rarely
> meaningful, because the units differ per type: `600` means 60.0 °C to a tank
> node, and a nonsensical load percent to a load node. `all` is appropriate for
> the binary types (door, handbrake) or when you genuinely want one number
> everywhere.

`type` values come from `enum sw_node_type` in `sensor_proto.h`:

| `type` | Node | Fitted on this wagon |
|---|---|---|
| 1 | bearing | ids 0 – 7 |
| 2 | load / tilt | ids 8 – 11 |
| **3** | **handbrake** | id 12 |
| **4** | **tank / cargo temperature** | ids 17 – 18 |
| 5 | door | ids 13 – 16 |
| 6 | air-brake pressure | *defined but not fitted* |

> ⚠ **Was documented wrong.** An earlier revision listed *"4 handbrake, 5 door,
> 6 tank temperature"* and skipped 3 entirely. **Handbrake is 3 and tank is 4.**
> A command built from the old list would target the wrong node type — `type:6`
> would match nothing (no air-brake node is fitted) and return `NOT_FOUND`,
> while `type:4` intending handbrake would hit the **tank** nodes.

**Response** — the count is how many nodes it was actually queued for:

```json
"d": { "cid":"n02", "cmd":"set_threshold", "res":"ok", "ec":null,
       "pl":{"applied":false,"state":"queued","nodes":8} }
```

**Only nodes that have been HEARD are counted.** The gateway needs the node
type to seal the BLE frame, so a provisioned but silent node is skipped — it is
not an error, it simply is not in `nodes`. **Always check `nodes` against what
you expected**; a wagon with eight bearings that reports `"nodes":6` has two
bearings that were not reachable, and they keep their old threshold.

If nothing matched at all — no node of that type provisioned, or none heard yet
— the response is `res:"err"`, `ec:"NOT_FOUND"`, with `"nodes":0`.

Each queued node is still delivered independently in its own ~4 s connectable
window, so a `"nodes":8` response means eight deliveries spread over the next
several minutes, not one broadcast.

### 6.7 Combining fleet scope with bulk nodes

The two axes multiply. Fanned out by the cloud to every wagon, this sets every
bearing on every wagon:

```json
{ "mt": "cmd", "cid": "n04", "cmd": "set_threshold", "scope": "bulk",
  "type": 1, "value": 800, "hyst": 50 }
```

Each wagon answers separately with its own `nodes` count. Unlike `ota_start`,
threshold fan-out is **not staggered** — it is a queue operation with no
network traffic, and delivery is already spread by the nodes' own windows.

### 6.8 The response is `queued`, not `applied`

```json
"d": { "cid":"n01", "cmd":"set_threshold", "res":"ok", "ec":null,
       "pl":{"applied":false,"state":"queued"} }
```

A sub-node is only reachable during a **~4 s connectable window, roughly every
10 minutes** — far longer than the broker will hold a response open. The
gateway queues the command and delivers it opportunistically when the observer
that is already scanning continuously next sees that node's window. No extra
radio time, no polling.

Confirmation that the node accepted the setting arrives later, in that node's
own uplink advert. `get_status` reports how many commands are still pending
delivery.

### 6.9 Three behaviours to design around

- **`NOT_FOUND` if the node has never been heard.** The gateway needs the node
  type to seal the BLE frame — it is authenticated in the AAD — and that only
  comes from an advert. A provisioned but silent node cannot be configured.
- **Last wins.** Queuing again for the same node **replaces** the pending
  command; there is no per-node queue depth.
- **No cancel, no read-back.** There is no command to withdraw a queued
  threshold or to read a node's current setting. Infer it from the node's
  uplinks.

### 6.10 `ota_start` for a sub-node

A sub-node image is delivered by the gateway over BLE. Add a `node` field to
`ota_start` and the gateway stages the image, then pushes it to that node the
next time the node opens a window.

```json
{
  "cmd": "ota_start",
  "cid": "c-501",
  "node": 17,
  "url": "https://fw.example.com/tank_b1t_1.3.0.bin",
  "ver": "1.3.0",
  "size": 163196
}
```

The gateway answers `queued`, not `applied` — the node is asleep and cannot be
reached synchronously. A fast config window is queued alongside so the node
opens windows every few seconds instead of every 10 minutes; it reverts on its
own deadline, so no command here can leave a node burning battery unattended.

#### One node, or a whole type

Both are supported. `node` addresses one; `type` addresses every node of that
type in the wagon's roster.

```json
{ "cmd": "ota_start", "cid": "c-503", "type": 1,
  "url": "https://fw.example.com/bearing_1.4.0.bin",
  "ver": "1.4.0", "size": 148992 }
```

That updates all eight bearings from **one** command — and, more importantly,
**one cellular download**. The image is fetched into the gateway's NOR once and
then streamed to each node over BLE. The modem leg is metered and slow; the
radio leg is neither, so eight bearings cost what one costs.

Targets come from the **roster**, not from what is audible at that instant. A
node asleep when the command arrives is still fitted and still needs the update;
it is reached when it next opens a window. Every target gets a fast config
window queued, not just the first — a node still on the 10-minute cadence when
its turn came would stall the queue behind it.

A node that fails is recorded and the campaign continues to the next. One
bearing out of range is not a reason to deny the other seven an update, and
re-publishing the same command later retries only the ones that missed it,
because the rest are already at the new version.

`"all": true` is still rejected: it would send one type's image to every sensor
on the wagon.

> **This reverses an earlier decision.** Previous revisions of this document
> explained that `NODE_ID` was compiled into each sub-node binary, that runtime
> provisioning had been "considered and not adopted", and that RDSO §5.1.13
> group-by-type FUOTA was therefore unsupported. The id now lives in NVS, seeded
> from the build and surviving every update, so the image carries no identity
> and one binary serves a whole node type. **19 sub-node binaries became 5.**
>
> The risk the old design guarded against — a node and the gateway disagreeing
> about which wheel a reading belongs to — is handled by keeping the id out of
> the image entirely, so an update cannot change it, and by refusing `node_id`
> on any scope broader than a single node (§6.11).

#### Updating the same node type on every wagon

A **fleet** operation. Publish once to `smartwagon/v1/all/dn/cmd` with a
`type`, and every gateway updates its own nodes of that type.

```json
{ "cmd": "ota_start", "cid": "c-504", "scope": "all", "type": 1,
  "url": "https://fw.example.com/bearing_1.4.0.bin",
  "ver": "1.4.0", "size": 148992 }
```

This works across wagons because the sub-node **wagon number** is in NVS too
(§5.12), so a bearing image is not tied to the wagon it was built for.

Add `"fitment"` to restrict a fleet roll-out to wagons of one fitment — see
§5.10. A gateway whose fitment differs answers `err/FITMENT_MISMATCH` and
downloads nothing.

#### How the transfer works

| Stage | Detail |
|---|---|
| Download | Gateway fetches over HTTP into the NOR `ota_partition` while the modem is already up |
| Integrity | CRC16 accumulated over every byte written |
| Delivery | Four-phase BLE transfer — `NL_CFG` → `NL_IMG_BEGIN` → `NL_IMG_CHUNK` → `NL_IMG_END` |
| Node side | Written straight into MCUboot's secondary slot via the flash-area API |
| Authenticity | **MCUboot signature verified at swap.** CRC16 is integrity only; the signature is what stops a forged image |
| Swap | `BOOT_UPGRADE_TEST` |
| Confirm | Node calls `boot_write_img_confirmed()` **after one complete successful cycle** |

That last row matters. `BOOT_UPGRADE_TEST` makes MCUboot revert on the next
reboot *unless* the running image confirms itself. The node confirms only after
it has read its sensors, sealed a payload and transmitted an advert — so an
image that crashes before that point reverts automatically, exactly as intended.

The campaign survives a gateway reset: state is persisted in FRAM at offset 128
under magic `0x4E4F5441`.

---

### 6.11 `node_id` — re-provision which node this is

```json
{ "mt":"cmd", "cid":"i1", "cmd":"set_threshold",
  "node": 17, "param":"node_id", "value": 12 }
```

Stores a new id in that node's NVS. It applies from the node's **next advert**,
without a reboot: the command arrives during a config window, and restarting
would drop the link before the gateway saw the response.

> ### ⚠ Requires an explicit `node`
>
> `type` and `all` are rejected with `BAD_PARAM`. Every other parameter is a
> value each node applies to itself, so a broad scope is exactly what it is for.
> An id is the opposite — it is what tells nodes **apart**. A type-scoped
> `node_id` would give all eight bearings the same id, and from that moment the
> gateway could not address any of them individually to undo it. Every bearing
> would need reflashing over SWD.

Range 0–19. A virgin node takes its first id from the build seed at manufacture;
this command is for re-provisioning a node you can already reach.

---

### 6.12 `cadence` — advertising period, over the air

```json
{ "mt":"cmd", "cid":"c1", "cmd":"set_threshold",
  "type": 1, "param":"cadence", "value": 30, "hyst": 1 }
```

`value` is the **quiet** period and `hyst` the **alarm** period, both in
seconds — the same two wire fields every other parameter uses, so the
individual / by-type / all-nodes scopes apply unchanged.

| Guard | Behaviour |
|---|---|
| Range | both clamped to **1 … 3600 s**. Below 1 s buys nothing over the alarm cadence and costs battery; above an hour the node cannot satisfy the gateway's 15-minute staleness check and is flagged DOWN continuously. |
| Ordering | the alarm period may never exceed the quiet one — an alarm reporting *less* often than normal traffic inverts the point of the fast cadence. |
| Persistence | stored in node NVS, so a cadence tuned over the air survives a reboot **and** a firmware update carrying different defaults. |
| Effect | immediate — the next sleep uses the new value. |

> **Battery consequence.** The §5.1.20 six-year projection is built on the
> **30 s** production figure. Dropping the quiet cadence to 5 s multiplies
> advertising charge by six and invalidates the life estimate. The clamp bounds
> the damage; it does not prevent it.

A `DEBUG_TRACE` build seeds **10 s** instead of 30 s so a node is visible on the
bench without waiting half a minute per advert. That is the seed only, not a
different mechanism.

---

### 6.13 `batt_mv` — the cell's two voltage references

```json
{ "mt":"cmd", "cid":"c41", "cmd":"set_threshold",
  "all": true, "param":"batt_mv", "value": 3550, "hyst": 3000 }
```

`value` is the **fresh** reference and `hyst` the **end-of-life knee**, both in
millivolts — the same two wire fields every other parameter uses, so the
individual / by-type / all-nodes scopes apply unchanged.

> **This is NOT a state-of-charge map.** Li-SOCl2 holds ≈3.6 V for ~95 % of its
> life and then falls off a cliff, so a voltage→percent curve reads ~100 % for
> years and then 0 within weeks — worse than no gauge, because it looks
> trustworthy. Sub-node SoC is **coulomb-counted** (§6.14). These two values feed
> only the parts of the gauge that voltage genuinely can answer.

| Value | What it actually does |
|---|---|
| `value` — **fresh** (3550) | Gate on the cell-replacement detector. A power-on reset counts as "somebody fitted a new cell" only if the cell *also* reads at least this, so a cell dying under brownout — which power-cycles the node too — can never reset the accumulator to 100 %. |
| `hyst` — **eol** (3000) | Independent end-of-life backstop. The coulomb counter drifts over a six-year life; the voltage knee does not, so crossing it pins the reported percentage down regardless of what the counter believes. |

| Guard | Behaviour |
|---|---|
| Range | both **2000 … 4000 mV** — an ER34615 is ≈3.6 V nominal and long dead below 2 V, so anything outside is a typo or volts-for-millivolts. |
| Separation | `fresh − eol` ≥ **100 mV**, so one cell cannot be simultaneously "fresh enough to be new" and "past end of life". |
| Persistence | node NVS; survives reboot and OTA. |
| Rejection | `INVALID_THRESHOLD` at the gateway; the node refuses independently. |

A wrong value here can only ever **weaken** the gauge, and neither failure
announces itself — the node keeps advertising happily. A `fresh` above the
cell's real voltage permanently disarms the replacement detector, so a genuinely
new cell reports the old one's depletion forever. An `eol` below the cell's
floor silently removes the end-of-life backstop.

---

### 6.14 `batt_cal_*` — calibrate the coulomb-counting gauge

A sub-node's percentage is not measured, it is **integrated**: every wake adds
`(current × time)` for each phase. The gauge is therefore only ever as accurate
as these seven constants — and all of them ship as **estimates**, marked
`MEASURE:` in `app_config.h`, because none can be known before a Nordic PPK2 has
been put in series with a real cell on real hardware.

Until now the only way to correct one was to open a sealed node and reflash it
over SWD.

```json
{ "mt":"cmd", "cid":"c42", "cmd":"set_threshold",
  "type": 1, "param":"batt_i_tx_ua", "value": 5200 }
```

Seven parameters share one opcode, distinguished by a selector carried in the
frame's previously-reserved `rsvd` byte — so the payload stays at six bytes and
reuses the existing CCM buffers unchanged.

| `param` | Unit | Seed | Accepted | What it covers |
|---|---|---|---|---|
| `batt_i_sleep_ua` | µA | 5 | 0 – 10000 | System ON idle, BLE idle |
| `batt_i_selfdisch_ua` | µA | 15 | 0 – 1000 | the cell's own leakage (~1 %/yr of 13 Ah ≈ 14.8 µA) |
| `batt_i_meas_ua` | µA | 1500 | 0 – 30000 | sensor rail on + conversion |
| `batt_t_meas_ms` | ms | 20 | **1** – 10000 | how long that phase lasts |
| `batt_i_tx_ua` | µA | 6000 | 0 – 30000 | mean over a non-connectable advertising burst |
| `batt_i_cfgwin_ua` | µA | 8000 | 0 – 30000 | the connectable config window |
| `batt_usable_mah` | mAh | 10000 | **100** – 30000 | derated usable capacity (13 Ah nameplate derated for load and winter temperature) |

`hyst` is unused — send `0` or omit it.

**Why the ceilings are where they are.** 13 Ah over a six-year life is **247 µA
average, total, across every phase**. A sleep current of 10 mA or a
self-discharge of 1 mA is not a tuning choice, it is a unit mix-up (mA sent where
µA was meant), and accepting it would drain the *modelled* cell to 0 % in days
while the real one sits full. Zero is allowed for the currents — a phase
genuinely measured as negligible should be settable to nothing — but not for
`batt_t_meas_ms` (a zero duration stops that phase being counted at all) or
`batt_usable_mah` (a zero capacity divides by zero).

> **This does not re-integrate the past.** The accumulator holds charge already
> counted using the *old* constants. A correction applies from the next wake
> onward and cannot undo error that has already accrued. Calibrate **before
> deployment** where possible, and treat this as a way to stop error growing for
> the remaining life — not a way to repair a year of it.

**One command per config window.** The gateway holds exactly one pending frame
per node, so a second command for the same node overwrites the first. Setting
all seven constants on one node is seven windows — up to ~70 minutes at the
normal 10-minute window period. Use `node_window` (§6.0) first to drop that to
a 5 s period while commissioning.

**Calibration procedure**

1. Set `BATT_DEBUG 0` — the debug printk's own UART and SAADC cost would
   otherwise be baked into everything you measure.
2. PPK2 in series with the cell; capture one full wake cycle.
3. Read the mean current of each phase off the trace.
4. Send each with this command, `node`-scoped for one unit or `type`-scoped once
   a batch has been characterised.

---

### 6.15 `ble_tx_dbm` — sub-node BLE output power

```json
{ "mt":"cmd", "cid":"c61", "cmd":"set_threshold",
  "type": 1, "param":"ble_tx_dbm", "value": 4 }
```

`value` is dBm, **signed** — negative levels are legal and are how you reduce a
node's range and power draw. `hyst` is unused.

This is the **node's** transmitter, which is what the gateway's RSSI measures.

| Guard | Behaviour |
|---|---|
| Range | **−40 … +8 dBm**, the hardware's range. Refused, not clamped, if outside. |
| Persistence | node NVS; survives reboot and OTA. |
| Effect | next advertising burst — the level belongs to the advertising set, which the node tears down between bursts, so it is re-applied each time. |
| Readback | the node prints `BLE TX: requested N dBm, controller selected M dBm`, flagging `(CLAMPED)` on a mismatch. |

> ⚠ **THIS SPENDS BATTERY, STEEPLY.** dBm is logarithmic; PA current is not.
> Measured against the 190 µA six-year ceiling at the 30 s production cadence:
>
> | Setting | Total | Life | |
> |---|---|---|---|
> | **0 dBm** (shipped default) | 134 µA | 8.5 yr | ✅ |
> | +4 dBm (≈1.5× radio) | ~191 µA | 6.0 yr | ⚠ on the line |
> | +8 dBm (≈2.7× radio) | 327 µA | 3.5 yr | ❌ **fails §5.1.20** |
>
> It fails this hard because the radio is **already ~84 % of the budget** at
> 0 dBm — sleep and self-discharge together are only 20 µA. Tripling the
> dominant term triples the total.
>
> **If you raise it, pay for it.** Doubling the quiet cadence to 60 s brings
> +8 dBm back to 155 µA and passes, at the cost of doubling worst-case alarm
> detection latency. Raising the power alone does not.

**Raising TX power will not fix a weak link caused by antennas.** Free-space
loss at 4 cm is only ~12 dB, so a node reading −64 dBm at that range is ~50 dB
down and +8 buys back 8 of them. Measure before spending budget.

---

## 7. Implementation status (firmware side)

| Command | Status |
|---|---|
| `get_config` | implemented |
| `get_status` | implemented (returns `applied:true`; data arrives on `up/hb`) |
| `pull_data` | implemented |
| `get_history` | implemented (count only; records replay via push) |
| `set_interval` | implemented, validated, persisted to FRAM |
| `set_gnss` | implemented, persisted to FRAM and PAIR513 |
| `set_threshold` (`impact_g`) | implemented, persisted to FRAM |
| `set_threshold` (node) | implemented — queued over BLE, delivered in the node's window |
| `set_batt` (gateway pack) | implemented, validated, persisted — §5.16 |
| `set_threshold` (`batt_mv`) | implemented, validated both ends, node NVS — §6.13 |
| `set_threshold` (`batt_cal_*`) | implemented, validated both ends, node NVS — §6.14 |
| `set_ble_tx` (gateway) | implemented, validated, persisted — §5.15b |
| `set_threshold` (`ble_tx_dbm`) | implemented, validated both ends, node NVS — §6.15 |
| `time_sync` | implemented (software clock; no battery-backed RTC yet) |
| `reboot` | implemented |
| `ota_start` (gateway) | implemented — HTTP → dfu_target → `boot_request_upgrade`, confirmed after a healthy online cycle |
| `ota_start` (sub-node) | implemented — staged to NOR, pushed over BLE, MCUboot-signed, confirmed after one good cycle. One node per command |
| `ota_status` | implemented |
| `provision_node` | intentionally `UNSUPPORTED` |
| MQTT downlink receive | **implemented** — `QMTSUB` + `+QMTRECV` parsing is wired and confirmed on hardware |
| uplink QoS | QoS 1 — publishes wait for the broker PUBACK, so store-and-forward only drops a record once truly delivered |

### 7.1 Persistence depends on the board

The gateway keeps one small `struct app_cfg` (~20 B): a working copy in SRAM
(read on the hot path, no per-report FRAM access) and a master copy in FRAM,
written only when a `set_*` actually changes a value. Sub-node thresholds
persist in the sub-node's own internal NVS (Zephyr settings), read once at boot
and written only on change.

**On a board with no FRAM fitted (e.g. the nRF54L15 DK) nothing persists.**
`config_save()` returns `void` and discards the write result, so a `set_*`
still answers `applied:true` — the value **is** active for the rest of that
power cycle — but the next boot logs `config: seeded defaults` and reverts to
the compile-time values in `app_config.h`. A boot that logs `config: loaded`
did restore stored settings. Check that line before concluding a `set_*` was
ignored.

## 8. Security notes

- All MQTT traffic should run over **TLS** (broker port 8883) in production;
  `cid` correlation alone is not authentication.
- **MQTT username and password are sent in cleartext in the CONNECT packet.**
  On a plain 1883 listener anyone observing the path can read them. Enabling
  authentication without TLS makes the exposure worse, not better — it adds a
  credential to what is already visible. RDSO §3.8 requires data "encrypted by
  the latest security standards available".
- Because commands can reboot or re-flash a wagon, treat broker credentials and
  the command channel as privileged. A future revision may add per-command
  signing so a compromised broker cannot issue rogue `ota_start` / `reboot`.
- The BLE sub-node link is separately protected by app-layer AES-CCM with a
  per-wagon key; that is independent of this MQTT command channel.

---
*Ref: RDSO WD-35-MISC-2024 · SmartWagon Telemetry Protocol Rev.1 (pv=1) · nRF54L15*
