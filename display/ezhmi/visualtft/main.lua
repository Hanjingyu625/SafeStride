-- VisualTFT M-series Lua. See docs/DISPLAY_KO.md for screens/controls/variables.
-- UTF-8 source; configure the project font/encoding to render Korean.
-- LCD is a Modbus SLAVE. This script never sends motor/control commands.
local PAGE = 1
local MINT, AMBER, CORAL, MUTED = 0x6FF9, 0xFDEE, 0xFB6E, 0x94B3
local ticks, last_change, last_heartbeat = 0, 0, nil
local received = false
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
    text(2, "손잡이: 확인 불가")
    text(3, "? 확인 불가", AMBER)
    text(4, reason, AMBER)
    text(5, "횡단보도 N/A")
    text(6, "신호 정보 없음")
    text(7, "경사 확인 불가")
    text(8, "--")
    text(9, reason, AMBER)
    text(10, "위치: N/A")
end

local function render(w)
    local valid, flags = w.valid, w.flags
    if bit(valid, 1) and w.speed ~= 65535 then
        text(1, string.format("%.2f km/h", w.speed / 100), MINT)
    else
        text(1, "-- km/h")
    end
    local hands = {[0]="손잡이: 놓음", "손잡이: 왼쪽", "손잡이: 오른쪽", "손잡이: 양손"}
    text(2, bit(valid, 2) and (hands[w.hands] or "손잡이: 확인 불가") or "손잡이: 확인 불가")

    -- ARMED alone can remain true during a controlled stop; braking,
    -- deadman, faults and terrain hazard must all be checked as well.
    if not bit(valid, 32) then
        text(3, "? 확인 불가", AMBER); text(4, "주행 상태 수신 없음", AMBER)
    elseif w.braking == 1 then
        text(3, "X 감속 / 제동", CORAL); text(4, "제동 동작 중", CORAL)
    elseif bit(flags, 4) or bit(flags, 8) or w.faults ~= 0 or w.walker >= 3 then
        text(3, "X 정지 상태", CORAL); text(4, "상태 / 오류 확인", CORAL)
    elseif w.hazard == 1 then
        text(3, "X 위험 감지", CORAL); text(4, "전방 장애물 / 낙차", CORAL)
    elseif w.walker == 2 and bit(flags, 1) and bit(flags, 2) then
        text(3, "O 주행 가능", MINT); text(4, "제어기 활성", MINT)
    else
        text(3, "X 대기", AMBER); text(4, "손잡이 / 제어 상태 확인", AMBER)
    end

    if not bit(valid, 4) or w.crosswalk == 0 then
        text(5, "횡단보도 N/A"); text(6, "신호 정보 없음")
    elseif bit(flags, 16) and bit(flags, 64) and w.crosswalk == 3 then
        text(5, "진입 가능", MINT)
    elseif w.crosswalk == 4 or w.crosswalk == 5 then
        text(5, bit(flags, 32) and "횡단 중: 주의" or "횡단 중", AMBER)
    elseif w.crosswalk == 6 then
        text(5, "횡단 완료 구간", MINT)
    else
        text(5, "횡단보도: 대기", AMBER)
    end
    if bit(valid, 4) and w.crosswalk ~= 0 then
        local distance = w.distance == 65535 and "--" or string.format("%.1f", w.distance / 10)
        local seconds = bit(flags, 64) and w.seconds ~= 65535 and tostring(w.seconds) or "--"
        text(6, "거리 " .. distance .. "m / 신호 " .. seconds .. "s")
    end

    if bit(valid, 8) then
        local pitch = w.pitch >= 32768 and w.pitch - 65536 or w.pitch
        pitch = pitch / 10
        local label, color = "평지", MINT
        if math.abs(pitch) >= 30 then label, color = "급경사", CORAL
        elseif pitch > 2 then label, color = "오르막", AMBER
        elseif pitch < -2 then label, color = "내리막", AMBER end
        text(7, label, color); text(8, string.format("%+.1f deg", pitch), color)
    else
        text(7, "경사 확인 불가"); text(8, "--")
    end
    text(9, w.hazard == 1 and "전방 위험 감지" or "데이터 수신 중", w.hazard == 1 and CORAL or MINT)
    text(10, "위치: N/A") -- No geocoded location exists in the system topics.
end

function on_init()
    ticks, last_change, last_heartbeat, received = 0, 0, nil, false
    change_screen(0) -- SafeStride logo screen
    set_enable(PAGE, 11, 0) -- DEV button stays disabled
    unknown("연결 대기")
    start_timer(0, 100, 1, 0)
end

function on_timer(timer_id)
    if timer_id ~= 0 then return end
    ticks = ticks + 1
    if ticks == 22 then change_screen(PAGE) end -- 2.2 second intro
    local w = {}
    for _, name in ipairs(names) do
        local value = get_variant("ss_" .. name)
        if type(value) ~= "number" then
            unknown("LCD 변수 설정 확인")
            return
        end
        w[name] = value
    end
    if w.version ~= 2 then unknown("화면 / 펌웨어 버전 확인"); return end
    if last_heartbeat == nil then
        last_heartbeat = w.heartbeat -- Initial register defaults are not live data.
    elseif last_heartbeat ~= w.heartbeat then
        last_heartbeat, last_change, received = w.heartbeat, ticks, true
    end
    -- Runs on the LCD, so Terrain power loss cannot leave a green frozen screen.
    if not received or ticks - last_change >= 10 or w.host_link ~= 1 then
        unknown("통신 끊김 / 연결 대기")
        return
    end
    render(w)
end
