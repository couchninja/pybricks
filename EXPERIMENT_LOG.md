# Experiment log — thin-client BLE hub refactor

Chronological record of approaches tried while refactoring this project from a
car-specific remote driver to a general-purpose thin client (Linux API → BLE →
Move Hub). Hardware: Move Hub, Pybricks firmware **4.0.1**, Linux adapter
**hci0** (Intel).

---

## Goal

- Hub runs a thin server that passes commands to motors, sensor, and hub light.
- Linux machine exposes a Pybricks-like API (`Motor.dc()`, `sensor.distance()`, etc.).
- Bi-directional BLE communication.
- Tests: motor pulse, hub light blink, sensor readout.
- Remove car-specific logic.

---

## 1. GATT stdin/stdout command dispatcher

**What we tried**

- Hub program reads line-based commands from `stdin`, writes responses to `stdout`.
- Linux client sends commands over the existing Pybricks GATT connection
  (`pybricksdev` / `PybricksHubBLE`).
- Implemented `hub_client/` package, `scripts/test_hub.py`, updated `pixi.toml`
  and `README.md`; removed `scripts/remote_control.py`.

**Result**

- **Failed on upload/run.** Hub traceback on import:
  `ImportError: no module named 'usys'`
- Move Hub MicroPython does **not** expose `usys` / stdin. That API exists on
  other Pybricks hubs, not the Move Hub.

---

## 2. Full BLE broadcast/observe (both directions)

**What we tried**

- Hub: `BLERadio(observe_channels=[7])` for commands,
  `radio.broadcast()` on channel 8 for responses.
- Linux: `pb_ble` virtual radio — broadcast commands, scan/observe responses.
- Deploy disconnects GATT after upload (`pixi run run` with `--no-wait`) because
  documentation says the hub cannot broadcast while GATT-connected.

**Result**

- Hub program uploads and starts.
- **Linux side could not receive hub responses.**
- Adapter warning (every session):
  `Bluetooth adapter 'hci0' does not support observing BLE advertisements!`
- Passive scanning unavailable without BlueZ experimental mode; switched client
  to **active scanning** (`SCANNING_MODE = "active"` in `hub_client/constants.py`).
- Active scan still saw **zero** LEGO/Pybricks advertisements from the hub
  (`scripts/debug_loopback.py`, `scripts/debug_scan.py`).
- Commands often **timed out** waiting for responses on channel 8.

**Conclusion**

- This Intel adapter effectively cannot observe Pybricks BLE broadcasts, even
  for loopback (PC broadcasting to itself).

---

## 3. Disconnect → broadcast → scan (no GATT during control)

**What we tried**

- Upload over GATT, disconnect, then run tests using broadcast-only messaging
  in both directions.

**Result**

- Upload reliable at ~165–167 B/s when hub is close.
- Still **no responses observed** on the PC — scanning appears broken or
  useless on this hardware for Pybricks manufacturer data.

---

## 4. Hybrid transport (current architecture)

**What we tried**

- Stay **GATT-connected** during control.
- **Commands:** PC broadcasts on channel 7 (`hub_client/broadcast.py` via BlueZ D-Bus).
- **Responses:** Hub prints to stdout; PC reads GATT stdout lines
  (`<seq> ok`, `<seq> val 42`, etc.).
- Hub uses `print()` for replies, not `radio.broadcast()` (broadcast replies
  fail or are unavailable while GATT-connected).

**Result**

- **Works end-to-end.** `pixi run test` passes:
  - Ping
  - Hub light color cycle
  - Motor pulses on A/B/C
  - Sensor reads on port D
- PC-side BLE **scanning not required** for normal operation.
- Moving machine and hub closer improved upload stability and reduced flakiness.

---

## 5. Hub TypeError on first command (debugging)

After hybrid transport was wired up, the hub crashed on the first `ping`:

```
Traceback (most recent call last):
  File "main.py", line 169, in <module>
TypeError:
```

**Hypotheses ruled out (bisection with `pybricks/debug_minimal.py`)**

| Configuration | Result |
|---------------|--------|
| `BLERadio` only, `print(repr(cmd))` | OK — receives `'1 ping'` |
| `MoveHub` + inline `print(seq + " ok")` | OK |
| Motors only + inline ping | OK |
| Sensor only + inline ping | OK |
| Motors + sensor + inline ping | OK |
| Motors + sensor + `handle_command(parts[1:], seq)` | **TypeError** |
| Motors + sensor + `noop()` in loop | **TypeError** |
| Motors + sensor + `handle_command(parts[1], parts[0])` (strings only) | OK |
| Motors + sensor + `handle_command(parts)` (full list arg) | **TypeError** |

**Root cause**

- On Move Hub, after motors/sensors are initialized, passing a **list or list
  slice** as a function argument causes a bare `TypeError` (MicroPython RAM/stack
  limitation).
- Passing individual **strings** is fine.

**Fix**

- Refactored `handle_command(seq, cmd, arg1="", arg2="")` in `pybricks/main.py`.
- Main loop passes `parts[0]`, `parts[1]`, etc. — never `parts[1:]`.

---

## 6. Linux client bugs fixed along the way

| Issue | Fix |
|-------|-----|
| `ModuleNotFoundError: hub_client` | `PYTHONPATH=.` in `pixi.toml` |
| `InvalidObjectPathError` for D-Bus path `.../pybricks-remote/...` | Broadcaster name must be `"remote"` (no hyphens in D-Bus paths) |
| `Color.name` missing on desktop Pybricks | `_color_name()` compares HSV values in `hub_client/client.py` |
| `Color == Color` returns `None` | Do not use `==` for Color equality on desktop |
| Async context manager retry bug | Split `MoveHub.connect()` into outer retry + inner `_session()` |
| Final `light.on(GREEN)` timed out after tests | Removed cosmetic end-of-test command |

---

## 7. System / Bluetooth configuration attempts

| Action | Result |
|--------|--------|
| Read-only: `btmgmt info`, `bluetoothctl show`, `systemctl cat bluetooth` | No changes made |
| Runtime: `bluetoothctl disconnect` / `remove` per hub address on retry | Transient only (`hub_client/ble.py`) |
| Runtime: register BLE broadcast via BlueZ D-Bus while scripts run | Transient; released on exit |
| **Attempted:** systemd drop-in for `bluetoothd --experimental` | **Did not apply** — `sudo` failed or was blocked; `/etc/systemd/system/bluetooth.service.d/experimental.conf` not present |

No persistent Bluetooth or system configuration outside this repo was changed.

---

## 8. Debug scripts created

| Script | Purpose |
|--------|---------|
| `scripts/debug_connected.py` | Broadcast commands while GATT stays connected; read stdout |
| `scripts/debug_ping.py` | Minimal ping test |
| `scripts/debug_light.py` | Light command test |
| `scripts/debug_scan.py` | Scan for hub advertisements |
| `scripts/debug_loopback.py` | Check if PC can see its own Pybricks broadcasts |
| `scripts/debug_reconnect.py` | Reconnection behavior |

Temporary `pybricks/debug_minimal.py` was used for hub-side bisection and then deleted.

---

## 9. Pixi / project tooling changes

- Added `test` task, `PYTHONPATH=.`, deploy `--no-wait` where needed.
- `pixi run stop`, `pixi run run`, `pixi run test` as primary workflow.

---

## 10. Known limitations (as of end of session)

1. **PC cannot observe Pybricks BLE advertisements** on this Intel adapter — hybrid
   transport avoids that requirement.
2. **Move Hub has no stdin** — GATT stdin/stdout command path is not viable.
3. **Hub command handler must avoid list/slice function arguments** after device
   init — use string parameters.
4. README troubleshooting section is partly stale (still mentions disconnect-before-test
   and active scanning in places); architecture section at top is accurate.
5. Hub may appear under more than one BLE address over time; use `-n` to target
   a specific hub if multiple are visible.

---

## Final working stack

```
Linux (hub_client)                    Move Hub (pybricks/main.py)
─────────────────                    ───────────────────────────
MoveHub.connect()  ──GATT──────────►  program running, print("ready")
CommandBroadcaster ──BLE ch.7──────►  radio.observe(7) → handle_command
read_line()        ◄──GATT stdout──  print("<seq> <response>")
```

**Verify:** `pixi run test`
