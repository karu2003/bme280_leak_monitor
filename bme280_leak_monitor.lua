--[[
  bme280_leak_monitor.lua
  Pixhawk 6C / ArduPilot Rover

  Leak watch in a sealed boat enclosure via BME280 (I2C):
    - reads humidity / temperature / pressure
    - on a sharp humidity rise (condensate / water) inside the compartment ->
      operator alert (GCS + buzzer) and RTL
    - optional: digital leak probe on an AUX pin as a fast backup channel

  About "raising Failsafe":
    Lua has no public write access to ArduPilot's internal failsafe
    (C++ core; AP_Notify/failsafe flags are not exposed for scripts).
    Practical equivalent from a script:
      1) switch to RTL (same outcome as most stock failsafes)
      2) repeating GCS STATUSTEXT at CRITICAL
      3) onboard buzzer (if GCS link is already gone)
    The event is also written to the dataflash log (LEAK).

  Check before flight:
    - I2C bus number (see BUS SCAN below)
    - BME280 address (0x76 or 0x77, SDO pin)
    - compensation follows the Bosch datasheet; compare against a known meter
--]]

---@diagnostic disable: need-check-nil

-- ===================== SETTINGS =====================
local I2C_BUS      = 2        -- Pixhawk 6C I2C2 / GPS2 (confirmed by scan)
local BME280_ADDR  = 0x76     -- SDO low; 0x77 if SDO is pulled high
local HUM_WARN_PCT = 70.0     -- humidity warning, %RH
local HUM_TRIGGER_PCT = 85.0  -- humidity for RTL / alert, %RH
local BASELINE_SAMPLES = 20   -- first samples averaged as the "dry" baseline
local DELTA_TRIGGER   = 15.0  -- also fire if humidity rose by N% from baseline

local USE_DIGITAL_PROBE = false -- true if a contact probe is on AUX
local LEAK_PIN = 55             -- AUX6 (only if USE_DIGITAL_PROBE)

local RTL_MODE_NUM = 11         -- Rover: HOLD=4, RTL=11
local ALERT_REPEAT_MS = 5000    -- how often to repeat the critical alert
local ALARM_TUNE = 'MFT200L8 O5 C C C C'  -- short buzzer trill

-- ===================== BUS SCAN (one-shot debug) =====================
-- Uncomment and run alone if you do not know the bus:
--[[
local scan_addr = 0
local scan_bus = i2c:get_device(I2C_BUS, 0)
scan_bus:set_retries(2)
function scan_update()
    scan_bus:set_address(scan_addr)
    if scan_bus:read_registers(0) then
        gcs:send_text(0, "I2C found at 0x" .. string.format("%02X", scan_addr))
    end
    scan_addr = scan_addr + 1
    if scan_addr == 127 then scan_addr = 0 end
    return scan_update, 50
end
return scan_update()
--]]

-- ===================== BME280 =====================
local dev = i2c:get_device(I2C_BUS, BME280_ADDR)
dev:set_retries(3)

local dig_T1, dig_T2, dig_T3
local dig_H1, dig_H2, dig_H3, dig_H4, dig_H5, dig_H6
local t_fine = 0

local function s16(v) if v >= 0x8000 then return v - 0x10000 else return v end end
local function s8(v)  if v >= 0x80   then return v - 0x100   else return v end end

local function read_u16(reg)
    local lo = dev:read_registers(reg)
    local hi = dev:read_registers(reg + 1)
    if not lo or not hi then return nil end
    return (hi << 8) | lo
end

local function read_calibration()
    dig_T1 = read_u16(0x88)
    dig_T2 = s16(read_u16(0x8A))
    dig_T3 = s16(read_u16(0x8C))

    dig_H1 = dev:read_registers(0xA1)
    dig_H2 = s16(read_u16(0xE1))
    dig_H3 = dev:read_registers(0xE3)

    local e4 = dev:read_registers(0xE4)
    local e5 = dev:read_registers(0xE5)
    local e6 = dev:read_registers(0xE6)
    dig_H4 = s16((e4 << 4) | (e5 & 0x0F))
    dig_H5 = s16((e6 << 4) | (e5 >> 4))
    dig_H6 = s8(dev:read_registers(0xE7))

    return dig_T1 and dig_H1 and dig_H4
end

local function configure_sensor()
    dev:write_register(0xF2, 0x01) -- ctrl_hum: humidity oversampling x1
    dev:write_register(0xF4, 0x27) -- ctrl_meas: temp/press oversampling x1, normal mode
    dev:write_register(0xF5, 0xA0) -- config: standby 1000ms, filter off
end

local function read_raw()
    local b = {}
    for i = 0, 7 do
        b[i] = dev:read_registers(0xF7 + i)
        if not b[i] then return nil end
    end
    local adc_P = (b[0] << 12) | (b[1] << 4) | (b[2] >> 4)
    local adc_T = (b[3] << 12) | (b[4] << 4) | (b[5] >> 4)
    local adc_H = (b[6] << 8)  | b[7]
    return adc_T, adc_P, adc_H
end

-- fixed-point compensation, Bosch reference driver (32-bit, masked for Lua int64)
local MASK32 = 0xFFFFFFFF

local function compensate_temperature(adc_T)
    local var1 = (((adc_T >> 3) - (dig_T1 << 1)) * dig_T2) >> 11
    local var2 = (((((adc_T >> 4) - dig_T1) * ((adc_T >> 4) - dig_T1)) >> 12) * dig_T3) >> 14
    t_fine = var1 + var2
    return ((t_fine * 5 + 128) >> 8) / 100.0 -- °C
end

local function compensate_humidity(adc_H)
    local v = t_fine - 76800
    v = (((adc_H << 14) - (dig_H4 << 20) - (dig_H5 * v)) + 16384) >> 15
    v = v & MASK32
    local x = ((((v * dig_H6) >> 10) * (((v * dig_H3) >> 11) + 32768)) >> 10) + 2097152
    x = (x * dig_H2 + 8192) >> 14
    v = ((v & MASK32) * (x & MASK32))
    local h = v - (((((v >> 15) * (v >> 15)) >> 7) * dig_H1) >> 4)
    if h < 0 then h = 0 end
    if h > 419430400 then h = 419430400 end
    return (h >> 12) / 1024.0 -- %RH
end

-- ===================== LEAK LOGIC =====================
local baseline_hum = nil
local baseline_sum = 0
local baseline_count = 0
local warned = false
local triggered = false
local last_alert_ms = 0

local function react_leak(current_hum)
    local now = millis()

    if not warned then
        gcs:send_text(3, string.format("HULL HUMIDITY WARN: %.1f%%", current_hum)) -- severity 3 = WARNING
        warned = true
    end

    if current_hum >= HUM_TRIGGER_PCT and not triggered then
        gcs:send_text(0, string.format("HULL LEAK - FAILSAFE - RTL (%.1f%%RH)", current_hum)) -- severity 0 = EMERGENCY
        notify:play_tune(ALARM_TUNE)
        vehicle:set_mode(RTL_MODE_NUM)
        triggered = true
        last_alert_ms = now
    end

    if triggered and (now - last_alert_ms) >= ALERT_REPEAT_MS then
        gcs:send_text(0, string.format("HULL LEAK ACTIVE (%.1f%%RH)", current_hum))
        notify:play_tune(ALARM_TUNE)
        last_alert_ms = now
    end
end

local function check_digital_probe()
    if not USE_DIGITAL_PROBE then return false end
    local leak = gpio:read(LEAK_PIN)
    return leak == 0 -- 0 = contacts closed by water (depends on pull / wiring)
end

-- ===================== INIT =====================
local initialised = false

local function init()
    local chip_id = dev:read_registers(0xD0)
    if chip_id ~= 0x60 then
        gcs:send_text(0, "BME280 not found on bus " .. tostring(I2C_BUS))
        return false
    end
    if not read_calibration() then
        gcs:send_text(0, "BME280 calibration read failed")
        return false
    end
    configure_sensor()
    if USE_DIGITAL_PROBE then
        gpio:pinMode(LEAK_PIN, 0)
    end
    gcs:send_text(6, "BME280 leak monitor initialised")
    return true
end

-- ===================== MAIN LOOP =====================
function update()
    if not initialised then
        initialised = init()
        return update, 1000
    end

    local adc_T, adc_P, adc_H = read_raw()
    if not adc_T then
        return update, 500
    end

    local temp_c = compensate_temperature(adc_T)
    local hum_pct = compensate_humidity(adc_H)

    if baseline_count < BASELINE_SAMPLES then
        baseline_sum = baseline_sum + hum_pct
        baseline_count = baseline_count + 1
        if baseline_count == BASELINE_SAMPLES then
            baseline_hum = baseline_sum / BASELINE_SAMPLES
            gcs:send_text(6, string.format("Baseline humidity: %.1f%%", baseline_hum))
        end
    else
        local delta = hum_pct - baseline_hum
        if hum_pct >= HUM_WARN_PCT or delta >= DELTA_TRIGGER then
            react_leak(hum_pct)
        end
    end

    if check_digital_probe() then
        react_leak(100.0) -- closed contact = highest priority
    end

    logger:write('LEAK', 'Temp,Hum,Base', 'ff', temp_c, hum_pct, baseline_hum or 0)

    return update, 1000
end

return update()
