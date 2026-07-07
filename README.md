# Pybricks remote hub

The Move Hub runs a thin BLE server that forwards commands directly to motors,
sensors, and the hub light. This Linux machine connects over BLE and exposes a
Pybricks-like API (`Motor.dc()`, `sensor.distance()`, `hub.light.on()`, etc.).

Communication uses a hybrid transport: commands are BLE-broadcast on channel 7,
responses come back over the GATT connection (stdout). The host stays connected
during control, so BLE scanning on the PC is not required.

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

## Deploy the hub program

Find your hub:

```sh
pixi run scan
```

Upload and start the thin-client program on the hub (disconnects when done):

```sh
pixi run run
```

Other deploy commands:

```sh
pixi run stop       # stop the program on the hub
pixi run upload     # upload without starting
pixi run dev        # interactive mode: re-upload after edits without reconnecting
```

Target a specific hub by name or address:

```sh
pixi run python scripts/ble_control.py -n "Move Hub" run pybricks/main.py
```

## Run tests from this machine

With the hub program deployed, run the built-in tests (blink light, pulse
motors, read sensor):

```sh
pixi run test
```

The test script connects over BLE, uploads the hub program if needed, and then
drives the hardware through the remote API.

## Remote API example

```python
import asyncio
from pybricks.parameters import Color, Direction, Port
from hub_client import MoveHub

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

`pybricks/main.py` maps ports as follows (edit to match your setup):

| Port | Device |
|------|--------|
| A | Motor (clockwise) |
| B | Motor (counter-clockwise) |
| C | Motor (counter-clockwise) |
| D | Color/distance sensor |

## Troubleshooting

- **Hub not found**: Turn the hub on, move it closer, and make sure it is not connected to the Pybricks app or another computer.
- **Upload fails or disconnects**: Run `pixi run stop`, wait a moment, then try `pixi run run` again. The script retries automatically (up to 5 times) and clears stale Bluetooth connections between attempts.
- **Connection fails immediately**: Power-cycle the hub, make sure no phone/tablet is connected to it, and try again.
- **Passive scan / BleakError on Linux**: Active scanning is used by default. If scanning still fails, ensure Bluetooth is enabled and no other app is using the adapter.
- **Commands time out**: The Move Hub cannot broadcast while connected over GATT. Run `pixi run run` (which disconnects automatically) or `pixi run stop` before using `pixi run test`. Close Pybricks Code and other BLE connections too.
