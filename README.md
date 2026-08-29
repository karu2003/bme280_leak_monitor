# BME280 leak monitor (ArduPilot Rover / Pixhawk 6C)

Lua scripts and host tools for a **BME280** humidity sensor on an ArduPilot Rover boat.

The sensor tracks humidity inside a sealed electronics enclosure. BME280 measures **relative humidity of air**, not standing water in the hull. Use a contact probe for flooding; BME280 is an early warning for condensation in the box.

Verified setup:

| Item | Value |
|---|---|
| Autopilot | Holybro Pixhawk 6C, ArduRover 4.7 |
| Sensor | BME280 on **GPS2** (I2C2) |
| Lua bus | `2` |
| I2C address | `0x76` |
| GCS | QGroundControl via **MAVProxy** (UDP 14550) |

## Upload scripts

Copy **one** `.lua` into `APM/scripts/` on the microSD:

- **SD card** — power off, mount the card on the PC, copy the file, eject, power on.
- **MAVFTP** — card stays in the Pixhawk; `py -3.12 tools/fc.py put …` or MAVProxy `ftp put`, or QGC / Mission Planner file transfer. Then `scripting restart` or reboot.

USB is MAVLink, not a flash drive. Details: [docs/TESTING.md](docs/TESTING.md#1-upload-scripts-sd-card-or-mavftp).

## Scripts on the SD card (`APM/scripts/`)

Keep **one** active script on the card. Do not install several at once.

| File | Role |
|---|---|
| [`i2c_scan.lua`](i2c_scan.lua) | One pass over GPS1/GPS2 to find the BME280 address |
| [`bme280_test.lua`](bme280_test.lua) | Read T/RH, no RTL. Current verification step |
| [`bme280_leak_monitor.lua`](bme280_leak_monitor.lua) | Humidity thresholds + RTL. Do not upload until the test is accepted |

## Host

```text
py -3.12 -m pip install -r requirements.txt
py -3.12 tools/fc.py serve          # MAVProxy: USB COM18 → QGC and CLI
py -3.12 tools/fc.py serve --tablet 192.168.0.50:14540   # + tablet Router UDP 14540
py -3.12 tools/fc.py ping
py -3.12 tools/fc.py test-bme
```

Connect QGC **only** to UDP `127.0.0.1:14550`. Do not open USB/Serial in QGC.

Full test procedure: **[docs/TESTING.md](docs/TESTING.md)**.

On the GCS: QGC shows the 5 s `BME280 T=… RH=…` line in **Messages** and the floats in **MAVLink Inspector**. Mission Planner Quick / HUD: **`MAV_Humidity`**, **`MAV_TempC`**. See [docs/TESTING.md §5](docs/TESTING.md#5-show-values-in-qgc-and-mission-planner).
