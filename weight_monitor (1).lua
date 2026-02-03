-- ============================
-- Power-Based Weight Monitor


local INTERCEPT = -24.10
local COEFFICIENT = 0.004130

local WEIGHT_LIMIT_KG = 10.0
local ALT_START = 5.0
local ALT_END = 20.0
local MIN_CLIMB_RATE = 0.5

local SAMPLE_PERIOD_MS = 100
local COLLECTION_TIME_S = 2
local REQUIRED_SAMPLES = math.floor(COLLECTION_TIME_S * 1000 / SAMPLE_PERIOD_MS)

local MIN_POWER_W = 50
local MAX_POWER_W = 50000

local MAX_REASONABLE_WEIGHT_KG = 100.0  -- sanity check upper bound

local monitoring = false
local collecting = false
local alerted = false
local samples = {}

local function reset_state()
    monitoring = false
    collecting = false
    alerted = false
    samples = {}
end

function update()

    if not arming:is_armed() then
        reset_state()
        return update, 500
    end

    local posD = ahrs:get_relative_position_D_home()
    if not posD then
        return update, SAMPLE_PERIOD_MS
    end
    local altitude = -posD

    local vel = ahrs:get_velocity_NED()
    if not vel then
        return update, SAMPLE_PERIOD_MS
    end
    local climb_rate = -vel:z()

    local in_band =
        altitude >= ALT_START and
        altitude <= ALT_END and
        climb_rate >= MIN_CLIMB_RATE

    -- Trigger sampling
    if in_band and not monitoring then
        gcs:send_text(6, "CLIMB DETECTED - starting weight estimation")
        monitoring = true
        collecting = true
        samples = {}
    end

    -- Collect samples
    if collecting then
        local voltage = battery:voltage(0)
        local current = battery:current_amps(0)

        --  Check if battery data is available
        if not voltage or not current then
            -- Battery data temporarily unavailable, skip this cycle
            return update, SAMPLE_PERIOD_MS
        end

        if voltage and current and current > 0 then
            local power = voltage * current
            if power >= MIN_POWER_W and power <= MAX_POWER_W then
                samples[#samples + 1] = power
            end
        end

        -- Stop ONLY when sample count reached
        if #samples >= REQUIRED_SAMPLES then
            collecting = false

            --  Validate we actually collected samples
            if #samples == 0 then
                gcs:send_text(4, "Weight estimation failed: no valid samples")
                monitoring = false
                return update, SAMPLE_PERIOD_MS
            end

            local sum = 0
            for i = 1, #samples do
                sum = sum + samples[i]
            end

            local avg_power = sum / #samples
            local weight = INTERCEPT + COEFFICIENT * avg_power

            -- Sanity check for weight estimation
            if weight < 0 or weight > MAX_REASONABLE_WEIGHT_KG then
                gcs:send_text(4, string.format("Weight estimation error: %.2f kg (out of range)", weight))
                monitoring = false
                return update, SAMPLE_PERIOD_MS
            end

            gcs:send_text(6,
                string.format("WEIGHT ESTIMATION COMPLETE"))
            gcs:send_text(6,
                string.format("Samples: %d", #samples))
            gcs:send_text(6,
                string.format("Average Power: %.1f W", avg_power))
            gcs:send_text(6,
                string.format("Estimated Weight: %.2f kg", weight))

            if weight > WEIGHT_LIMIT_KG and not alerted then
                gcs:send_text(3, "OVERWEIGHT WARNING")
                alerted = true
            end

            monitoring = false
        end
    end

    return update, SAMPLE_PERIOD_MS
end

gcs:send_text(6, "Power-based weight monitor loaded")
return update, 1000