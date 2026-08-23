# Smart Wagon — Downlink Command Reference (`dn/cmd`)

Server → gateway control channel for the SmartWagon Telemetry Protocol (Rev.1).
Controller: nRF54L15 (QFN52). This document is the contract the **server side** builds against.

---

## 1. Topics

| Direction | Topic | QoS | Retain |
|---|---|---|---|
| Command (server → gateway) | `smartwagon/v1/{wgn}/{gw}/dn/cmd` | 1 | **false** |
| Response (gateway → server) | `smartwagon/v1/{wgn}/{gw}/up/resp` | 1 | false |

- `{wgn}` = wagon number, `{gw}` = gateway id (both derived from the wagon number).
- **Never publish a command with `retain=true`** — a retained command would re-fire on every reconnect. Commands are delivered through the broker's **persistent session** queue instead, so they are held for the gateway until it next connects.

## 2. Timing — read this first (Class-A device)

The gateway is a sleeping, uplink-first (Class-A) device. **It only processes queued commands when it wakes and connects** — i.e. right after one of its own uplinks. So a command may sit queued at the broker from **seconds up to ~12 hours** (the stopped-wagon reporting interval) before it is executed and answered.

If you need it sooner, the pattern is: publish the command, then the server has no push path to wake the wagon — you simply wait for the next scheduled wake. (A future `wake`/paging mechanism is out of scope for Rev.1.) Design the server UI to treat responses as **asynchronous** and correlate them by `cid`.

## 3. Common request envelope

Every command is a single JSON object:

```json
{ "mt": "cmd", "cid": "a1b2", "cmd": "<command>", "...": "command-specific fields" }
```

| Field | Type | Req | Notes |
|---|---|---|---|
| `cid` | string (≤31) | yes | Correlation id. Echoed back in the response so you can pair request↔reply. Make it unique per outstanding command. |
| `cmd` | string | yes | One of the commands in §5. |

## 4. Common response envelope

The gateway answers each command on `up/resp`:

```json
{ "cid": "a1b2", "cmd": "set_interval", "result": "ok", "applied": true }
```

| Field | Type | Notes |
|---|---|---|
| `cid` | string | Same `cid` as the request. |
| `cmd` | string | Echo of the command. |
| `result` | string | `"ok"` or `"err"`. |
| `reason` | string | Present only when `result="err"` (see codes below). |
| *(payload)* | — | Command-specific fields (documented per command). |

**Reason codes** (`result="err"`): `UNSUPPORTED` (unknown command), `BAD_PARAM` (missing/invalid field), `BUSY` (e.g. an OTA is already running), `NOT_FOUND` (target node not provisioned), `FAILED` (execution error).

---

## 5. Commands

### 5.1 `set_threshold` — change an alarm limit

Sets one alarm threshold held by the gateway (e.g. impact-g). Node-held thresholds (e.g. a bearing's over-temp point) are addressed with `node` and are relayed to that sub-node the next time it opens a config window.

**Request**

```json
{ "mt": "cmd", "cid": "t01", "cmd": "set_threshold", "param": "impact_g", "value": 4.0, "hyst": 0.5 }
```

| Field | Type | Req | Notes |
|---|---|---|---|
| `param` | string | yes | `impact_g`, `tilt_deg`, `tank_hi_c`, `tank_lo_c`, `bearing_temp_c`, … |
| `value` | number | yes | New threshold value (units implied by `param`). |
| `hyst` | number | no | Release hysteresis (same units). |
| `node` | int | no | Target sub-node id for a node-held threshold; omit for a gateway threshold. |

**Response**

```json
{ "cid": "t01", "cmd": "set_threshold", "result": "ok", "applied": true, "param": "impact_g", "value": 4.0 }
```

Persisted in FRAM, so it survives reboots.

---

### 5.2 `set_interval` — change reporting cadence

```json
{ "mt": "cmd", "cid": "i01", "cmd": "set_interval", "moving_s": 600, "idle_s": 43200 }
```

| Field | Type | Req | Notes |
|---|---|---|---|
| `moving_s` | int (s) | no* | Heartbeat period while moving (default 600 = 10 min). |
| `idle_s` | int (s) | no* | Heartbeat period while stopped (default 43200 = 12 h). |

\* Send either or both; at least one is required.

**Response**

```json
{ "cid": "i01", "cmd": "set_interval", "result": "ok", "applied": true, "moving_s": 600, "idle_s": 43200 }
```

The new cadence takes effect at the next `arm_schedule()` (i.e. after the current report). Persisted in FRAM.

---

### 5.3 `set_gnss` — change GNSS behaviour

```json
{ "mt": "cmd", "cid": "g01", "cmd": "set_gnss", "enable": true, "fix_timeout_s": 90, "constellation": "navic_gps" }
```

| Field | Type | Req | Notes |
|---|---|---|---|
| `enable` | bool | no | `false` = stop taking fixes (report last-known only) to save power. |
| `fix_timeout_s` | int (s) | no | Max time to wait for a fix per report (default 90). |
| `constellation` | string | no | `navic`, `gps`, `navic_gps`, or `auto`. |

**Response**

```json
{ "cid": "g01", "cmd": "set_gnss", "result": "ok", "applied": true }
```

Persisted in FRAM.

---

### 5.4 `get_status` — report now, out of schedule

No parameters. The gateway publishes an **immediate heartbeat** on `up/hb` and returns a status snapshot on `up/resp`.

**Request**

```json
{ "mt": "cmd", "cid": "s01", "cmd": "get_status" }
```

**Response**

```json
{
  "cid": "s01", "cmd": "get_status", "result": "ok",
  "fw": "1.2.0", "state": "running",
  "uptime_s": 84213, "batt": 92,
  "fix_valid": true, "lat": 19.0760, "lon": 72.8777,
  "nodes": 4, "nodes_down": 0,
  "moving_s": 600, "idle_s": 43200
}
```

---

### 5.5 `ota_start` — start a firmware update

Begins download of a **signed** image for the **gateway only**. Applies via MCUboot with signature + version + downgrade checks and auto-revert. There is no sub-node firmware-update path: a `target` naming a sub-node is rejected. (The image is staged in MCUboot's secondary slot in internal flash; the external NOR holds telemetry only, so an update can never erase buffered data.)

**Request**

```json
{
  "mt": "cmd",
  "cid": "o01", "cmd": "ota_start",
  "target": "gateway",
  "url": "https://ota.macnman.com/gw/sw_gateway_1.3.0.bin",
  "ver": "1.3.0",
  "size": 245760,
  "sha256": "9f2c…e17a"
}
```

| Field | Type | Req | Notes |
|---|---|---|---|
| `target` | string | yes | `gateway`, or `node`. |
| `node` | int | for node | Sub-node id when `target="node"`. |
| `node_type` | int | for node | Sub-node type (see `enum sw_node_type`) when `target="node"`. |
| `url` | string | yes | HTTPS location of the signed image. |
| `ver` | string | yes | Target version (SemVer; must be ≥ current if downgrade-prevention is on). |
| `size` | int (bytes) | yes | Image size, for progress + sanity check. |
| `sha256` | string | no | Hex digest; verified before swap (in addition to the MCUboot signature). |

**Response** (accepted)

```json
{ "cid": "o01", "cmd": "ota_start", "result": "ok", "state": "downloading", "pct": 0 }
```

**Response** (rejected)

```json
{ "cid": "o01", "cmd": "ota_start", "result": "err", "reason": "BUSY" }
```

---

### 5.6 `ota_status` — query an update in progress

```json
{ "mt": "cmd", "cid": "o02", "cmd": "ota_status" }
```

**Response**

```json
{ "cid": "o02", "cmd": "ota_status", "result": "ok",
  "state": "downloading", "pct": 37, "ver": "1.3.0" }
```

`state` ∈ `idle`, `downloading`, `verifying`, `ready`, `applying`, `error`. On `error`, an `err` string describes the cause.

---

### 5.7 `reboot` — restart the gateway

The gateway answers **first**, waits briefly so the response is delivered, then performs a cold reset.

```json
{ "mt": "cmd", "cid": "r01", "cmd": "reboot", "mode": "cold" }
```

| Field | Type | Req | Notes |
|---|---|---|---|
| `mode` | string | no | `cold` (default). |

**Response**

```json
{ "cid": "r01", "cmd": "reboot", "result": "ok" }
```

---

### 5.8 `time_sync` — set the RTC (bonus)

Sets the gateway clock from server time (also done automatically at boot from GNSS UTC; use this to correct drift or when no fix is available).

```json
{ "mt": "cmd", "cid": "c01", "cmd": "time_sync", "epoch": 1786060800 }
```

| Field | Type | Req | Notes |
|---|---|---|---|
| `epoch` | int | yes | UTC seconds since 1970-01-01. |

**Response**

```json
{ "cid": "c01", "cmd": "time_sync", "result": "ok", "applied": true }
```

---

## 6. Implementation status (firmware side)

| Command | Status in current firmware |
|---|---|
| `get_status` | implemented |
| `ota_start`, `ota_status` | **image download + flash + MCUboot swap implemented** (HTTP-over-EC200 → dfu_target → `boot_request_upgrade`, with auto-revert on unconfirmed boot). MQTT signals the URL; HTTP transfers the bytes. Needs bench validation. **Prerequisite:** the trigger arrives on `dn/cmd`, whose MQTT *receive* path (`QMTSUB`/`+QMTRECV`) is still to be wired. |
| `reboot` | implemented |
| MQTT downlink receive | **not yet wired** — `ec200_mqtt_poll_cmd` is a stub, so no downlink command is actually received from the broker yet (all handlers exist; the pipe does not). This is the next piece to implement. |
| `set_interval` | **implemented** — updates `moving_s`/`idle_s`, persists to FRAM (`struct app_cfg`), re-arms the schedule. Validated ranges: `moving_s` 30–86400, `idle_s` 60–604800. |
| `set_gnss` | **implemented** — updates enable / fix-timeout / constellation, persists to FRAM. |
| `set_threshold` | **implemented for the gateway-held `impact_g`** (0.5–16.0 g, persisted to FRAM). Node-held thresholds (`bearing_temp_c`, `tilt_deg`, `tank_*`) are accepted by the schema but their relay to the sub-node's own NVS is a follow-up; they currently return `err/BAD_PARAM`. |
| `time_sync` | **implemented** — sets the gateway software clock (also disciplined by each GNSS fix; free-runs between fixes so `ts` is valid without a live fix). A hardware RTC that persists across power loss is a future addition. |
| uplink QoS | **QoS 1** — publishes wait for the broker PUBACK, so store-and-forward only drops a record once truly delivered. |

Persistence model: the gateway keeps one small `struct app_cfg` (~20 B) — the working copy in SRAM (read on the hot path, no per-report FRAM access) and the master copy in FRAM, written only when a `set_*` actually changes a value. Sub-node thresholds persist in the sub-node's own internal NVS (Zephyr settings), read once at boot and written only on change.

## 7. Security notes

- All MQTT traffic should run over **TLS** (broker port 8883) in production; `cid` correlation alone is not authentication.
- Because commands can reboot or re-flash a wagon, treat the broker credentials and the command channel as privileged. A future revision may add per-command signing so a compromised broker cannot issue rogue `ota_start`/`reboot`.
- The BLE sensor link (sub-node → gateway) is separately protected by app-layer AES-CCM (see the encryption design); that is independent of this MQTT command channel.

---
*Ref: RDSO WD-35-MISC-2024 · SmartWagon Telemetry Protocol Rev.1 (pv=1) · nRF54L15 (QFN52)*
