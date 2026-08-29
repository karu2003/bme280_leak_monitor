# How to test the BME280 on Pixhawk 6C

Testing is done from a PC over USB. QGroundControl only watches telemetry through MAVProxy. Mission Planner is not required.

Bench reference (indoor, August 2026):

- scan: `I2C I2C2-GPS2 0x76 BME280`
- test: **T ≈ 25.7 °C**, **RH ≈ 44.7 %** — later confirmed: T and RH track room air and a breath on the sensor.

If temperature stays roughly 10–40 °C, humidity 20–80 %RH, and neither jumps by tens of units every sample, the sensor and compensation are healthy.

---

## 0. Hardware

1. Pixhawk 6C with a microSD card inserted.
2. BME280: 3.3 V, GND, SDA/SCL on **GPS2** (not GPS1).
3. Address `0x76` (SDO to GND). If SDO is tied to 3.3 V the address is `0x77` — change `BME280_ADDR` in the script.
4. USB to the PC. Two COM ports: usually **COM18** = MAVLink, COM19 = second CDC.

Pixhawk 6C Lua I2C numbering:

| Lua bus | Bus | Connector |
|---|---|---|
| 0 | I2C4 | onboard (baro, compass) — do not scan unless you need to |
| 1 | I2C1 | GPS1 |
| 2 | I2C2 | GPS2 ← BME280 |

Autopilot parameter: `SCR_ENABLE = 1` (reboot if you just turned it on).

---

## 1. Upload scripts: SD card or MAVFTP

ArduPilot loads Lua from the microSD path **`APM/scripts/`** (FAT, case-insensitive). Keep **one** `.lua` file there while testing.

USB on Pixhawk is MAVLink, not a mass-storage disk. You either pull the card, or copy files with **MAVFTP**.

After any upload, the running script does not change until you **reboot** the flight controller or run **`scripting restart`**.

### SD card (no telemetry)

Use this when USB/MAVFTP is not up, or you want a guaranteed copy on the card.

1. Power off the Pixhawk. Do not pull the card while it is writing logs.
2. Take out the microSD and mount it on the PC.
3. Create `APM/scripts/` if it is missing (`APM/LOGS` is usually already there).
4. Copy **one** of:
   - `i2c_scan.lua`
   - `bme280_test.lua`
   - `bme280_leak_monitor.lua`
5. Delete the other two `.lua` files from `APM/scripts/` so they do not run together.
6. Eject the card, put it back, power on.
7. Set `SCR_ENABLE = 1` in QGC Parameters if it is still 0, then reboot once.

Check: QGC Messages should show `BME280 test: bus2 0x76 OK` or `I2C scan idle` / leak init, depending on which file you copied.

### MAVFTP (card stays in the Pixhawk)

MAVFTP is the MAVLink file protocol. The card stays in the flight controller. USB must be free for MAVProxy (QGC on UDP only).

**CLI (preferred here)**

```text
py -3.12 tools/fc.py serve
py -3.12 tools/fc.py ping
py -3.12 tools/fc.py ls APM/scripts
py -3.12 tools/fc.py put bme280_test.lua APM/scripts/bme280_test.lua
py -3.12 tools/fc.py rm APM/scripts/i2c_scan.lua
py -3.12 tools/fc.py restart
```

`put` overwrites the remote file. `test-bme` / `scan` do put + delete extras + restart for you.

**MAVProxy console** (the window from `serve`)

```text
ftp list APM/scripts
ftp put bme280_test.lua APM/scripts/bme280_test.lua
ftp rm APM/scripts/i2c_scan.lua
scripting restart
```

**QGroundControl**

1. Connect via UDP 14550 (through MAVProxy).
2. Analyze Tools → **MAVLink File Transfer** (or equivalent MAVFTP view).
3. Open `/APM/scripts/`.
4. Upload the `.lua` file; delete the scripts you do not want.
5. Restart scripting: reboot, or ask the CLI: `py -3.12 tools/fc.py restart`.

QGC does not always expose `scripting restart`. A reboot always reloads `APM/scripts/`.

**Mission Planner** (optional)

1. Connect (USB only if MAVProxy is **not** running).
2. Config → **MAVFTP** → `APM/scripts/` → Upload.
3. Full Parameter List: `SCR_ENABLE = 1`.
4. Restart scripting or reboot.

Do not open Mission Planner Serial and MAVProxy on the same COM port at the same time.

### After upload

| Action | When |
|---|---|
| `py -3.12 tools/fc.py restart` | `SCR_ENABLE` was already 1 at boot |
| Reboot | first time you set `SCR_ENABLE = 1`, or scripting heap is exhausted |

`ls APM/scripts` should list exactly the file you intend to run.

---

## 2. Link: MAVProxy + QGC

**Only MAVProxy** may own USB. QGC uses UDP.

```text
py -3.12 tools/fc.py serve
```

A MAVProxy console opens. Leave it running.

| Who | Where |
|---|---|
| MAVProxy | `--master=COM18` |
| QGC | Comm Link **UDP**, listen `14550` |
| CLI | `tcp:127.0.0.1:14551` |
| Tablet Router | extra `--tablet <TABLET_WIFI_IP>:14540` |

To feed **MAVLink Router** on the tablet (preset **QGC + Widget**, UDP in `14540`):

```text
py -3.12 tools/fc.py serve --tablet 192.168.0.50:14540
```

If a hub is already running without `--tablet`, stop that MAVProxy window and start again. Widget listens on the tablet at UDP `14551` and must have **Humidity** / **Temp** enabled in Choose parameters.

In QGC disable Serial/COM for this Pixhawk. Otherwise QGC steals USB, MAVProxy dies, and the CLI stops pinging.

Check the link:

```text
py -3.12 tools/fc.py ping
```

Expected: `Heartbeat sys=1 … device=tcp:127.0.0.1:14551`.

QGC Fly View should show the Rover/Boat. Messages should not spam `bus 0 / next pass`.

TCP `5760` is taken by WSL on this machine — the CLI uses `14551` on purpose.

---

## 3. I2C scan (find the sensor)

Use this after wiring or address changes. Only `i2c_scan.lua` on the card.

```text
py -3.12 tools/fc.py scan --seconds 20
```

One pass over GPS1 and GPS2, internal bus skipped, then idle (so QGC does not read “bus” aloud).

**Pass**

```text
I2C I2C1-GPS1 done n=0
I2C I2C2-GPS2 0x76 BME280
I2C I2C2-GPS2 done n=1
I2C scan idle
```

**Fail**

- both `done n=0` — no ACK: wiring, 3.3 V, GPS1/GPS2 swapped, address not 0x76/0x77;
- a device ACKs but is not `BME280` — chip id at 0xD0 is not `0x60` (BMP280 is `0x58`).

Remove `i2c_scan.lua` after the scan (or run `test-bme`, which deletes it).

---

## 4. BME280 read test

No RTL, no leak thresholds. Only `bme280_test.lua` on the card.

```text
py -3.12 tools/fc.py test-bme --seconds 22
```

The command removes scan/leak scripts, uploads the test, runs `scripting restart`, and prints STATUSTEXT plus `NAMED_VALUE_FLOAT`.

**What “good” looks like**

1. `BME280 test: bus2 0x76 OK` — chip id 0x60, calibration read.
2. About every 5 s: `BME280 T=25.76C RH=44.6%` (your numbers, same order of magnitude).
3. `Humidity` and `TempC` named floats about once per second.
4. Values are stable; breathe on the sensor — RH should rise in a few seconds, then fall.

In QGC: Messages → the same INFO lines. INFO is usually quieter than Warning; turn off Enable audio if needed.

In Mission Planner: Quick / HUD User Items → `MAV_Humidity`, `MAV_TempC`.

**Bad signs**

| Symptom | Check |
|---|---|
| `BME280 missing bus 2` | I2C drop, wrong bus/address |
| `T=0` and `RH=0`, or RH stuck at 0/100 | calibration, burst read, power |
| T jumps −40…+80 | broken compensation or cable noise |
| No STATUSTEXT in CLI, but QGC has it | CLI is not sending GCS heartbeats; retry via MAVProxy, do not take COM18 |
| `Scripting: unable to allocate memory` | too many restarts — reboot the FC |

---

## 5. Show values in QGC and Mission Planner

`bme280_test.lua` must be running. It sends:

| What | MAVLink | Name |
|---|---|---|
| Line every 5 s | STATUSTEXT | `BME280 T=25.74C RH=44.7%` |
| Live numbers ~1 Hz | NAMED_VALUE_FLOAT | `Humidity`, `TempC` |

Connect GCS through **MAVProxy** (QGC = UDP 14550). Do not attach a second program to COM18.

### QGroundControl

QGC **does not** put Lua named floats on the Fly View instrument strip (no `MAV_Humidity` picker). Use text and the inspector.

**Messages (easiest)**

1. Fly View → **Messages** (speech bubble / console).
2. Wait up to 5 s for `BME280 T=…C RH=…%`.
3. If the list is quiet: `py -3.12 tools/fc.py restart`, confirm `BME280 test: bus2 0x76 OK`.

Turn off **Application Settings → General → Enable audio** if it reads every line aloud.

**MAVLink Inspector (live floats)**

1. **Analyze** → **MAVLink Inspector**.
2. Find **NAMED_VALUE_FLOAT**.
3. Fields: `name` = `Humidity` or `TempC`, `value` = the reading.

There is no stock QGC widget for those names. A custom QGC build would be required to pin them on the HUD.

### Mission Planner

MP maps each named float to **`MAV_` + name**.

**Quick**

1. Flight Data → **Quick**.
2. Double-click a cell (or right-click → select items).
3. Enable **`MAV_Humidity`** and **`MAV_TempC`**.
4. If you only see `customfield0`: switch tab and back, or wait for the first named-float packet after connect.

**HUD**

1. Right-click the HUD → **User Items**.
2. Tick the same `MAV_Humidity` / `MAV_TempC`.

**Status / graphs**

- **Status**: full list of current values, including the `MAV_*` fields.
- **Tuning** graphs: plot `MAV_Humidity` / `MAV_TempC` once they have appeared.

If Quick stays at 0 but Inspector/Messages are fine, MP is not bound to the same sysid as the autopilot (do not send named floats from a companion as a different component).

---

## 6. What this test does not cover

`bme280_test.lua` does **not** enable RTL or thresholds. The leak monitor is a separate step.

[`bme280_leak_monitor.lua`](../bme280_leak_monitor.lua) already has the verified `I2C_BUS = 2`, `BME280_ADDR = 0x76`, `RTL_MODE_NUM = 11` (Rover mode 4 is HOLD, not RTL).

Before a flight upload:

1. Put the sensor in a **sealed dry** box, or 70/85 %RH and +15 delta will false-trigger.
2. Only the leak script on the card — no test, no scan.
3. First run disarmed; watch the baseline in Messages. Do not expect RTL on the bench unless you want it.

---

## CLI commands

```text
py -3.12 tools/fc.py ports
py -3.12 tools/fc.py ping
py -3.12 tools/fc.py serve              # MAVProxy in a new window
py -3.12 tools/fc.py ls APM/scripts
py -3.12 tools/fc.py put FILE.lua APM/scripts/FILE.lua
py -3.12 tools/fc.py rm APM/scripts/i2c_scan.lua
py -3.12 tools/fc.py restart            # reload scripts from the SD card
py -3.12 tools/fc.py monitor --seconds 20
py -3.12 tools/fc.py scan
py -3.12 tools/fc.py test-bme
```

The same actions exist in the MAVProxy console: `ftp put …`, `scripting restart`.
