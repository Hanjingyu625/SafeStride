-- VisualTFT M-series Lua. See docs/DISPLAY_KO.md for screens/controls/variables.
-- ASCII-only display strings avoid missing glyphs in the bundled LCD fonts.
-- LCD is a Modbus SLAVE. This script never sends motor/control commands.
local PAGE = 1
local BUILD_TAG = "BUILD 2"
local MINT, AMBER, CORAL, MUTED = 0x6FF9, 0xFDEE, 0xFB6E, 0x94B3
local ticks, last_change, last_heartbeat = 0, 0, nil
local received = false
local page_entered = false
local names = {"version", "heartbeat", "valid", "speed", "hands", "crosswalk",
    "seconds", "distance", "pitch", "tof", "hazard", "walker", "braking",
    "faults", "host_link", "flags"}

local function bit(value, mask)
    return math.floor(value / mask) % 2 == 1
end

local function text(id, value, color)
    set_text(PAGE, id, value)
    set_fore_color(PAGE, id, color or MUTED)
end

local function unknown(reason)
    text(1, "-- km/h")
    text(2, "Grip: unavailable")
    text(3, "? UNAVAILABLE", AMBER)
    text(4, reason, AMBER)
    text(5, "Crosswalk N/A")
    text(6, "No signal data")
    text(7, "Slope N/A")
    text(8, "--")
    text(9, BUILD_TAG .. " | " .. reason, AMBER)
    text(10, "Location: N/A")
end

local function render(w)
    local valid, flags = w.valid, w.flags
    if bit(valid, 1) and w.speed ~= 65535 then
        text(1, string.format("%.2f km/h", w.speed / 100), MINT)
    else
        text(1, "-- km/h")
    end
    local hands = {[0]="Grip: released", "Grip: left", "Grip: right", "Grip: both"}
    text(2, bit(valid, 2) and (hands[w.hands] or "Grip: unavailable") or "Grip: unavailable")

    -- ARMED alone can remain true during a controlled stop; braking,
    -- deadman, faults and terrain hazard must all be checked as well.
    if not bit(valid, 32) then
        text(3, "? UNAVAILABLE", AMBER); text(4, "No drive status", AMBER)
    elseif w.braking == 1 then
        text(3, "X BRAKING", CORAL); text(4, "Brake active", CORAL)
    elseif bit(flags, 4) or bit(flags, 8) or w.faults ~= 0 or w.walker >= 3 then
        text(3, "X STOPPED", CORAL); text(4, "Check state / faults", CORAL)
    elseif w.hazard == 1 then
        text(3, "X HAZARD", CORAL); text(4, "Obstacle / drop-off", CORAL)
    elseif w.walker == 2 and bit(flags, 1) and bit(flags, 2) then
        text(3, "O READY", MINT); text(4, "Controller armed", MINT)
    else
        text(3, "X STANDBY", AMBER); text(4, "Check grip / control", AMBER)
    end

    if not bit(valid, 4) or w.crosswalk == 0 then
        text(5, "Crosswalk N/A"); text(6, "No signal data")
    elseif bit(flags, 16) and bit(flags, 64) and w.crosswalk == 3 then
        text(5, "ENTRY ALLOWED", MINT)
    elseif w.crosswalk == 4 or w.crosswalk == 5 then
        text(5, bit(flags, 32) and "CROSSING: CAUTION" or "CROSSING", AMBER)
    elseif w.crosswalk == 6 then
        text(5, "CROSSING COMPLETE", MINT)
    else
        text(5, "WAIT AT CROSSWALK", AMBER)
    end
    if bit(valid, 4) and w.crosswalk ~= 0 then
        local distance = w.distance == 65535 and "--" or string.format("%.1f", w.distance / 10)
        local seconds = bit(flags, 64) and w.seconds ~= 65535 and tostring(w.seconds) or "--"
        text(6, "Dist " .. distance .. "m / Signal " .. seconds .. "s")
    end

    if bit(valid, 8) then
        local pitch = w.pitch >= 32768 and w.pitch - 65536 or w.pitch
        pitch = pitch / 10
        local label, color = "LEVEL", MINT
        if math.abs(pitch) >= 30 then label, color = "STEEP", CORAL
        elseif pitch > 2 then label, color = "UPHILL", AMBER
        elseif pitch < -2 then label, color = "DOWNHILL", AMBER end
        text(7, label, color); text(8, string.format("%+.1f deg", pitch), color)
    else
        text(7, "Slope N/A"); text(8, "--")
    end
    text(9, w.hazard == 1 and "Front hazard detected" or "Receiving data", w.hazard == 1 and CORAL or MINT)
    text(10, "Location: N/A") -- No geocoded location exists in the system topics.
end

local function update_display()
    if not page_entered then
        if ticks < 2 then return end
        change_screen(PAGE) -- Two-second SafeStride intro.
        page_entered = true
    end
    local w = {}
    for _, name in ipairs(names) do
        local value = get_variant("ss_" .. name)
        if type(value) ~= "number" then
            unknown("Check LCD variables")
            return
        end
        w[name] = value
    end
    if w.version ~= 2 then unknown("Display / firmware mismatch"); return end
    if last_heartbeat == nil then
        last_heartbeat = w.heartbeat
        -- Non-zero heartbeat plus a live host link cannot be an untouched default.
        if w.heartbeat ~= 0 and w.host_link == 1 then
            last_change, received = ticks, true
        end
    elseif last_heartbeat ~= w.heartbeat then
        last_heartbeat, last_change, received = w.heartbeat, ticks, true
    end
    -- Runs on the LCD, so Terrain power loss cannot leave a green frozen screen.
    if not received or ticks - last_change >= 1 or w.host_link ~= 1 then
        unknown("Link lost / waiting")
        return
    end
    render(w)
end

function on_init()
    ticks, last_change, last_heartbeat, received = 0, 0, nil, false
    page_entered = false
    change_screen(0) -- SafeStride logo screen
    set_enable(PAGE, 11, 0) -- DEV button stays disabled
    unknown("Waiting for link")
end

function on_systick()
    -- VisualTFT calls this callback automatically once per second.
    ticks = ticks + 1
    update_display()
end
