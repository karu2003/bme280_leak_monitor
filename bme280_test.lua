--[[
  bme280_test.lua
  Pixhawk 6C — BME280 readout only, no RTL/failsafe.

  Bus: I2C2 (GPS2) = Lua bus 2, address 0x76 (confirmed by scan).
  QGC: Messages (INFO every 5 s). Named floats: Humidity, TempC.
  Do not keep i2c_scan.lua or bme280_leak_monitor.lua on the card at the same time.
--]]

---@diagnostic disable: need-check-nil

local I2C_BUS = 2
local BME280_ADDR = 0x76
local REPORT_MS = 5000

local dev = i2c:get_device(I2C_BUS, BME280_ADDR)
dev:set_retries(3)

local dig_T1, dig_T2, dig_T3
local dig_H1, dig_H2, dig_H3, dig_H4, dig_H5, dig_H6
local t_fine = 0.0
local last_report_ms = 0
local ready = false

local function s16(v)
    if not v then return nil end
    if v >= 0x8000 then return v - 0x10000 end
    return v
end

local function s12(v)
    v = v & 0xFFF
    if v >= 0x800 then return v - 0x1000 end
    return v
end

local function s8(v)
    if v >= 0x80 then return v - 0x100 end
    return v
end

local function read_u16le(reg)
    local raw = dev:read_registers(reg, 2)
    if not raw then return nil end
    return raw[1] | (raw[2] << 8)
end

local function init()
    local id = dev:read_registers(0xD0)
    if id ~= 0x60 then
        gcs:send_text(4, string.format("BME280 missing bus %d id=0x%02X", I2C_BUS, id or 0))
        return false
    end

    dig_T1 = read_u16le(0x88)
    dig_T2 = s16(read_u16le(0x8A))
    dig_T3 = s16(read_u16le(0x8C))
    dig_H1 = dev:read_registers(0xA1)
    dig_H2 = s16(read_u16le(0xE1))
    dig_H3 = dev:read_registers(0xE3)
    local hum = dev:read_registers(0xE4, 3)
    if not (dig_T1 and dig_H1 and hum) then
        gcs:send_text(4, "BME280 calib read failed")
        return false
    end
    dig_H4 = s12((hum[1] << 4) | (hum[2] & 0x0F))
    dig_H5 = s12((hum[3] << 4) | (hum[2] >> 4))
    dig_H6 = s8(dev:read_registers(0xE7))

    -- ctrl_hum must be written before ctrl_meas
    dev:write_register(0xF2, 0x01) -- hum os x1
    dev:write_register(0xF4, 0x27) -- temp/press os x1, normal
    dev:write_register(0xF5, 0xA0) -- standby 1000 ms
    gcs:send_text(6, "BME280 test: bus2 0x76 OK")
    return true
end

local function compensate_temperature(adc_T)
    local var1 = (adc_T / 16384.0 - dig_T1 / 1024.0) * dig_T2
    local var2 = ((adc_T / 131072.0 - dig_T1 / 8192.0) ^ 2) * dig_T3
    t_fine = var1 + var2
    return t_fine / 5120.0
end

local function compensate_humidity(adc_H)
    local h = t_fine - 76800.0
    h = (adc_H - (dig_H4 * 64.0 + dig_H5 / 16384.0 * h))
        * (dig_H2 / 65536.0 * (1.0 + dig_H6 / 67108864.0 * h * (1.0 + dig_H3 / 67108864.0 * h)))
    h = h * (1.0 - dig_H1 * h / 524288.0)
    if h > 100.0 then h = 100.0 end
    if h < 0.0 then h = 0.0 end
    return h
end

function update()
    if not ready then
        ready = init()
        return update, 1000
    end

    local raw = dev:read_registers(0xF7, 8)
    if not raw then
        return update, 500
    end

    local adc_T = (raw[4] << 12) | (raw[5] << 4) | (raw[6] >> 4)
    local adc_H = (raw[7] << 8) | raw[8]
    local temp_c = compensate_temperature(adc_T)
    local hum_pct = compensate_humidity(adc_H)

    gcs:send_named_float("Humidity", hum_pct)
    gcs:send_named_float("TempC", temp_c)

    local now = millis():toint()
    if (now - last_report_ms) >= REPORT_MS then
        gcs:send_text(6, string.format("BME280 T=%.2fC RH=%.1f%%", temp_c, hum_pct))
        last_report_ms = now
    end

    return update, 1000
end

return update()
