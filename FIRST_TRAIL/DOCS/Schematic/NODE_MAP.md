# Smart Wagon — Canonical Fleet Node-ID Map

Purpose: keep `node_id` ↔ physical position **consistent across every wagon** so
the MQTT data is readable fleet-wide, even when different wagons omit different
sensors.

## The three rules

1. **`node_id` is explicit** (the first field in `nodes.c`, and `NODE_ID` in each
   sub-node's `app_config.h`). Removing a node from a wagon does **not** renumber
   the others. The two must always match for a given physical node.
2. **Never reuse an id for a different position/type.** A wagon that lacks a
   sensor simply **omits** that id (leaves a gap) — it must **not** compact the
   numbering. So id 8 is *always* "B1LT-L load/tilt" on every wagon, present or
   absent.
3. **The server identifies a sensor by `(wgn, typ, pos)` — never by `node_id`.**
   `node_id` is only the compact on-air / gateway-cache address. `pos` + `typ`
   are the position-semantic identity that mean the same thing on every wagon.

## Canonical allocation (reserved ranges by type)

| id range | type | positions (this wagon) |
|---|---|---|
| 0 – 7  | `SW_TYPE_BEARING`   | B1-A1-L, B1-A1-R, B1-A2-L, B1-A2-R, B2-A1-L, B2-A1-R, B2-A2-L, B2-A2-R |
| 8 – 11 | `SW_TYPE_LOAD_TILT` | B1LT-L, B1LT-R, B2LT-L, B2LT-R |
| 12     | `SW_TYPE_HANDBRAKE` | HB |
| 13 – 16| `SW_TYPE_DOOR`      | B1D-L, B1D-R, B2D-L, B2D-R |
| 17 – 18| `SW_TYPE_TANK_TEMP` | B1T, B2T |
| 19 +   | *(spare)*           | reserve for future sensor types |

To add a new sensor **type** to the fleet, give it its own reserved id range at
the end (19+) and use it on every wagon that carries it — do not overlap an
existing range.

## Examples

- A flatbed wagon with **no doors**: omit ids 13–16. Bearings stay 0–7, temp
  stays 17–18. The server still sees `pos:"B1-A1-L"` identically to any other
  wagon.
- A wagon with **only bearings + handbrake**: ids 0–7 and 12 present; 8–11 and
  13–18 simply absent. No renumbering.

## Capacity note

`node_id` indexes the gateway cache (`SW_MAX_NODES` in `ble_sensors.h`, currently
20) and is 1 byte on air. Keep the **fleet-wide** master list of distinct
positions under `SW_MAX_NODES`. If the fleet ever needs more than 20 distinct
positions, raise `SW_MAX_NODES` (and re-flash gateways).

---
*Ref: SmartWagon Telemetry Protocol Rev.1 · nRF54L15 (QFN52)*
