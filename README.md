# Cursor setup
On raspberry + ubuntu, when the agent calls the terminal, the commands would hang indefenitely
- Fixed by installing zsh: "sudo apt install zsh"
- Alternatively: Cursor settings -> Agents -> Legacy Terminal Tool -> on
  - needs window reload (and possibly new chat)
  - drawbacks: less isolation, no sandbox and no newer cursor features
  - can only be set for all of cursor

# raspberry hardware
## sudo GPIO
- In order to use the pins (gpiod) without sudo:
  - ```sudo usermod -aG dialout "$USER"```
  - reboot
## WiFi
- raspberry pi 4b does not want to reliably connect to lego brick over BLE if wifi is turned on: connect to raspberry over ethernet. It does not seem needed to turn off the wifi.
- update: I reconnected over wifi, and BLE connectivity seems fine; keep an eye on it
- update: I had the problem again, and a reboot did not fix it
  - instead I connected over ethernet
  - and then had to do: "pixi run wifi-off && pixi run wifi-on"
    - perhaps I can run that still connected to wifi
## Keep it Light
- The raspberry can hang if it is pushed too hard:
  - Cursor plugins: only install python
  - Don't run too many agents simultaneously

# Pybricks remote hub

The Move Hub runs a thin BLE server that forwards commands directly to motors,
sensors, and the hub light. This Linux machine connects over BLE and exposes a
Pybricks-like API (`Motor.dc()`, `sensor.distance()`, `hub.light.on()`, etc.).

Communication stays on the GATT connection the whole time: commands are written
to hub stdin, responses come back on stdout. BLE advertising/scanning on the PC
is not required, and BlueZ does not need `--experimental`.

## Flash Pybricks firmware

1. Go to https://code.pybricks.com/ in Chrome (does not work in Brave)
2. Click "Install Pybricks Firmware" and follow the instructions
   - Don't hold the button too long. The hub status light will turn off for a second while flash memory is erased, then start flashing again — release the button then.
3. The hub is now controllable over BLE.

## Setup

Install [pixi](https://pixi.sh), then from this directory:

```sh
pixi install
```

Bluetooth must be enabled on this machine. On Linux, your user needs permission to use Bluetooth (usually automatic on desktop distros).

### Autostart navigator on boot

Install and enable the systemd unit (starts after Bluetooth on reboot):

```sh
pixi run navigator-autostart
```

Disable and stop (no longer starts on boot):

```sh
pixi run navigator-autostart-off
```

Output appends to `logs/navigator.log`. Manage with:

```sh
pixi run navigator-start
pixi run navigator-stop
pixi run navigator-status
pixi run navigator-logs
```

## Deploy the hub program

Find your hub:

```sh
pixi run ble-scan
```

Upload and start the thin-client program on the hub (disconnects when done):

```sh
pixi run hub-run
```

Other deploy commands:

```sh
pixi run hub-stop       # stop the program on the hub
pixi run hub-upload     # upload without starting
pixi run hub-dev        # interactive mode: re-upload after edits without reconnecting
```

Target a specific hub by name or address:

```sh
pixi run python pybricks_client/ble_control.py -n "Move Hub" run pybricks_hub/main.py
```

## Run tests from this machine

With the hub program deployed, run the built-in tests (blink light, pulse
motors, read sensor):

```sh
pixi run test-hub
```

The test script connects over BLE, uploads the hub program if needed, and then
drives the hardware through the remote API.

## Remote API example

```python
import asyncio
from pybricks.parameters import Color, Direction, Port
from pybricks_client import MoveHub

async def main():
    async with MoveHub.connect() as hub:
        motor = hub.motor(Port.A, Direction.CLOCKWISE)
        sensor = hub.color_distance_sensor(Port.D)

        await hub.light.on(Color.BLUE)
        await motor.dc(25)
        await asyncio.sleep(0.3)
        await motor.stop()
        print(await sensor.distance())

asyncio.run(main())
```

## Hub wiring defaults

`pybricks_hub/main.py` maps ports as follows (edit to match your setup):

| Port | Device |
|------|--------|
| A | Built-in Motor |
| B | Built-in Motor |
| C | External Motor |
| D | Color/distance sensor |

## Troubleshooting

- **Hub not found**: Turn the hub on, move it closer, and make sure it is not connected to the Pybricks app or another computer.
- **Upload fails or disconnects**: Run `pixi run hub-stop`, wait a moment, then try `pixi run hub-run` again. The script retries automatically (up to 5 times) and clears stale Bluetooth connections between attempts.
- **Connection fails immediately**: Power-cycle the hub, make sure no phone/tablet is connected to it, and try again.
- **Commands time out**: Make sure the hub program is the latest upload (`pixi run hub-run` or reconnect with upload). Close Pybricks Code and other BLE connections to the hub.
