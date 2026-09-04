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
| **Attempted:** systemd drop-in for `bluetoothd --experimental` | Tried during broadcast debugging; **not required** for GATT stdin. Drop-in removed; stock `bluetoothd` confirmed working. |

No persistent Bluetooth or system configuration outside this repo is required.

---

## 8. Debug scripts

Broadcast-era scripts under `scripts/debug_*.py` were removed after the GATT
stdin switch. Use `pixi run test` / `pixi run nav` for verification.

---

## 9. Pixi / project tooling changes

- Added `test` task, `PYTHONPATH=.`, deploy `--no-wait` where needed.
- `pixi run stop`, `pixi run run`, `pixi run test` as primary workflow.
- Added `navigator` task (`hub_client/navigator/navigator_main.py`) for
  application-level hub control.

---

## 10. `motor.run_until_stalled` / `motor.stall` (navigator)

**What we tried**

- Expose `Motor.run_until_stalled()` on the Linux client and dispatch it from
  `pybricks/main.py`.
- First attempt used the full Pybricks name and keyword-style wire args:
  `motor.run_until_stalled B 100 duty_limit=30`.
- Refactored `handle_command()` to `*args` and `*parts[2:]` to pass optional
  keyword args.

**Result**

- `pixi run test` still passed; `pixi run nav` failed.
- **Payload too large:** `motor.run_until_stalled B 100 duty_limit=30` encodes
  to ~48 bytes; `pb_ble` rejects anything over **26 bytes**:
  `ValueError: Payload too large: 48 bytes (maximum is 26 bytes)`.
  The limit is in the Pybricks BLE broadcast format (31-byte adv packet minus
  overhead), not specific to this adapter.
- **`*args` regression:** Reintroduced the Move Hub `TypeError` from §5 when
  `handle_command(seq, cmd, *args)` replaced the fixed-arity handler. Reverted
  to `arg1`–`arg4` and explicit dispatch in the main loop.
- **Client timeout:** `run_until_stalled` blocks on the hub until the motor
  stalls; the default 10 s `_CommandSession.call()` timeout is too short.
  `run_until_stalled` now uses a 120 s timeout.

**Fix**

- Short wire name: `motor.stall` (not `motor.run_until_stalled`).
- Positional args only: `motor.stall <port> <speed> [<duty_limit>] [<then>]`.
  Example on the wire: `2 motor.stall B 100 30` (~23 bytes encoded).
- Hub calls `motor.run_until_stalled(speed, then, duty_limit)` with positional
  args (no keyword args on MicroPython).
- `pixi run nav` works end-to-end (motor B stalls, returns angle).

---

## 11. Known limitations (as of GATT stdin switch)

1. **PC cannot register or observe Pybricks BLE advertisements** on this Intel
   adapter (`Invalid Parameters 0x0d`) — do not rely on BlueZ LE advertising.
2. **Move Hub has no blocking `input()` / `usys.stdin`** — use
   `pybricks.tools.read_input_byte()` and assemble lines on the hub.
3. **Hub command handler must avoid list/slice function arguments** after device
   init — use string parameters.
4. Hub may appear under more than one BLE address over time; use `-n` to target
   a specific hub if multiple are visible.

---

## 12. GATT stdin transport (current)

BLE broadcast for commands failed permanently on this machine. Final design:

```
Linux (hub_client)                    Move Hub (pybricks/main.py)
─────────────────                    ───────────────────────────
MoveHub.connect()  ──GATT──────────►  program running, print("ready")
write_line()       ──GATT stdin────►  read_input_byte() → handle_command
read_line()        ◄──GATT stdout──  print("<seq> <response>")
```

Removed: `hub_client/broadcast.py`, `pb_ble_import.py`, `constants.py`,
`pybricks-ble` / `dbus-fast` deps, and broadcast-era `scripts/debug_*.py`.

BlueZ `--experimental` is **not** needed; verified working with stock
`bluetoothd` after removing the experimental systemd drop-in.

**Verify:** `pixi run test`, `pixi run nav`
