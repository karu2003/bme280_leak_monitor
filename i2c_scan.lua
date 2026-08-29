--[[
  i2c_scan.lua
  Pixhawk 6C — one pass, then idle (so QGC does not speak STATUSTEXT).

  bus 0 = I2C4 internal (skipped by default: baro/compass)
  bus 1 = I2C1 GPS1
  bus 2 = I2C2 GPS2
--]]

---@diagnostic disable: need-check-nil

local SCAN_INTERNAL = false
local FIRST_ADDR = 0x08
local LAST_ADDR  = 0x77
local ADDRS_PER_TICK = 4

local BUS_INFO = {
    [0] = "I2C4-int",
    [1] = "I2C1-GPS1",
    [2] = "I2C2-GPS2",
}

local buses = {}
local first_bus = SCAN_INTERNAL and 0 or 1
for bus = first_bus, 2 do
    local ok, dev = pcall(function()
        return i2c:get_device(bus, 0)
    end)
    if ok and dev then
        dev:set_retries(2)
        buses[#buses + 1] = { n = bus, dev = dev, label = BUS_INFO[bus] or tostring(bus) }
    end
end

if #buses == 0 then
    gcs:send_text(6, "I2C scan: no external buses")
    return
end

local bus_i = 1
local addr = FIRST_ADDR
local found_on_bus = 0
local started = false

local function identify(dev, a)
    local id = dev:read_registers(0xD0)
    if id == 0x60 then return "BME280" end
    if id == 0x58 then return "BMP280" end
    if id == 0x50 then return "BMP388" end
    if id == 0x61 then return "BME680" end
    if a == 0x0C then
        local wai = dev:read_registers(0x00)
        if wai == 0x10 then return "IST8310" end
    end
    return "ACK"
end

local function begin_bus()
    found_on_bus = 0
    addr = FIRST_ADDR
end

function idle()
    return idle, 60000
end

function update()
    if not started then
        started = true
        begin_bus()
    end

    local b = buses[bus_i]
    for _ = 1, ADDRS_PER_TICK do
        b.dev:set_address(addr)
        if b.dev:read_registers(0) then
            found_on_bus = found_on_bus + 1
            gcs:send_text(6, string.format("I2C %s 0x%02X %s", b.label, addr, identify(b.dev, addr)))
        end
        addr = addr + 1
        if addr > LAST_ADDR then
            gcs:send_text(6, string.format("I2C %s done n=%d", b.label, found_on_bus))
            bus_i = bus_i + 1
            if bus_i > #buses then
                gcs:send_text(6, "I2C scan idle")
                return idle, 60000
            end
            begin_bus()
            return update, 200
        end
    end
    return update, 20
end

return update()
