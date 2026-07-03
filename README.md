# Pybricks robot (BLE control)

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

## BLE control

Find your hub:

```sh
pixi run scan
```

Upload and run the robot program (stops any running program first):

```sh
pixi run run
```

Other commands:

```sh
pixi run stop       # stop the program on the hub
pixi run download   # upload without starting
pixi run dev        # interactive mode: re-upload after edits without reconnecting
```

Target a specific hub by name or address:

```sh
pixi run python scripts/ble_control.py -n "Move Hub" run pybricks/main.py
```

## Live remote control (arrow keys)

The robot listens for drive commands on BLE broadcast channel 7 using
[pybricks-ble](https://github.com/fkleon/pybricks-ble). This works while the
hub program is running and does not need a GATT connection.

1. Deploy the program: `pixi run run`
2. Wait until it disconnects (or press Ctrl+C after upload finishes)
3. Drive with arrow keys:

```sh
pixi run remote
```

Arrow keys set left/right motor power. Release a key to stop. Press `Q` or `Esc` to quit the remote.

Remote control overrides autonomous driving while keys are held (hub light turns yellow).
When you release the keys, the robot resumes its normal behavior.

**Tip:** Broadcasting works best when the hub is not connected to Pybricks Code or
`pybricksdev` at the same time.

## Troubleshooting

- **Hub not found**: Turn the hub on, move it closer, and make sure it is not connected to the Pybricks app or another computer.
- **Upload fails or disconnects**: Run `pixi run stop`, wait a moment, then try `pixi run run` again. The script retries automatically (up to 5 times) and clears stale Bluetooth connections between attempts.
- **Connection fails immediately**: Power-cycle the hub, make sure no phone/tablet is connected to it, and try again. The hub may take a few attempts to accept a new BLE connection while a program is running.
- **Program keeps running**: Pybricksdev has no built-in stop command; use `pixi run stop` instead.
