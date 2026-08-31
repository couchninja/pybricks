# Agent notes

- Before connecting to or testing the LEGO Move Hub (e.g. `pixi run navigator`, `pixi run test-hub`, hub-upload/hub-run), ask the user to put the brick in receiving mode and wait for confirmation. Do not attempt a hub connection until they say it is ready.
- Hub control uses GATT stdin/stdout only (`write_line` / `read_input_byte`). Do not reintroduce BLE advertising or `pybricks-ble` — this Intel adapter cannot register LE advertisements.
- Stock BlueZ is enough (`bluetoothd` without `--experimental`). Do not enable BlueZ experimental mode for this project.
