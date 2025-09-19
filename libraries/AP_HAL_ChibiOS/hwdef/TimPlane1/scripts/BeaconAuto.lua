-- Automatically turn off the beacon
-- Turn on and flash when armed

--[[

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   Follow in Plane
   Support follow "mode" in plane. This will actually use GUIDED mode with 
   a scripting switch to allow guided to track the vehicle id in FOLL_SYSID
   Uses the AP_Follow library so all of the existing FOLL_* parameters are used
   as documented for Copter, + add 3 more for this script
   FOLLP_EXIT_MODE - the mode to switch to when follow is turned of using the switch
   FOLLP_FAIL_MODE - the mode to switch to if the target is lost
   FOLLP_TIMEOUT - number of seconds to try to reaquire the target after losing it before failing
   FOLLP_OVRSHT_DEG - if the target is more than this many degrees left or right, assume an overshoot
   FOLLP_TURN_DEG - if the target is more than this many degrees left or right, assume it's turning
--]]

SCRIPT_VERSION = "4.6.0-015"
SCRIPT_NAME = "Auto Beacon"
SCRIPT_NAME_SHORT = "Beacon"

MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

CHANNEL_DELAY = 250 -- how long the Beacon channel needs to be at it's PWM value for the switch to work
ON_OPTION = 302     -- This options either leaves the beacon permanently on or state switched
STATE_OPTION = 303  -- This option cycles through the states, but only if the beacon is "off"
SERVO_FUNCTION = 94

-- allow a manual switch to override the auto beacon
local rc_state_switch = rc:find_channel_for_option(STATE_OPTION)
local rc_on_switch = rc:find_channel_for_option(ON_OPTION)
--local EKF_TYPE = Parameter('AHRS_EKF_TYPE')
local sw_pos_old = 0

BeaconState = { Flashing = 0, Solid = 1, SlowBlink = 2, FastBlink = 3, Off = 4}

local refresh_rate = 25         -- in Hz
local now_ms = millis()
local now_wait_ms = now_ms
local beacon_current = BeaconState.Flashing
local beacon_desired = BeaconState.Flashing    -- This is how it should start up

local is_armed = false

local output_current = -1
function SwitchChannel()
    if output_current == 0 then
        output_current = 1
    else
        output_current = 0
    end
end

local output_old = 999
function OutputChannel()
    --function SRV_Channels:set_output_norm(function_num, value) end
    if output_current == -1 then
        SRV_Channels:set_output_pwm(SERVO_FUNCTION, 0)  -- no pulses for flashing
    else
        SRV_Channels:set_output_norm(SERVO_FUNCTION, output_current)
    end
    if output_current ~= output_old then
        --gcs:send_text(0, string.format("Output %d pwm %d", output_current, SRV_Channels:get_output_pwm(SERVO_FUNCTION)))
        output_old = output_current
    end
end

function NextBeacon()
    if beacon_current == BeaconState.Flashing then
        beacon_current = BeaconState.Solid
        SwitchChannel()
    elseif beacon_current == BeaconState.Solid then
        beacon_current = BeaconState. SlowBlink
        SwitchChannel()
    elseif beacon_current == BeaconState.SlowBlink then
        beacon_current = BeaconState.FastBlink
        SwitchChannel()
    elseif beacon_current == BeaconState.FastBlink then
        beacon_current = BeaconState.Off
        SwitchChannel()
    elseif beacon_current == BeaconState.Off then
        beacon_current = BeaconState.Flashing
        output_current = -1     -- special case for flashing!
    end

    -- gcs:send_text(MAV_SEVERITY.ALERT, SCRIPT_NAME_SHORT .. " Next Beacon " .. BeaconName(beacon_current))
end

function WaitForBeacon()
    if beacon_current == beacon_desired then
        now_wait_ms = now_ms
        return
    end
    if now_ms > (now_wait_ms + 2000) then
        NextBeacon()
        now_wait_ms = now_ms
    end
end

function BeaconName(beacon_state)
    if beacon_state == BeaconState.Flashing then
        return "Flashing"
    elseif beacon_state == BeaconState.Off then
        return "Off"
    elseif beacon_state == BeaconState.FastBlink then
        return "Fast Blink"
    elseif beacon_state == BeaconState.SlowBlink then
        return "Slow Blink"
    elseif beacon_state == BeaconState.Solid then
        return "Solid"
    end
    return ""
end

-- set the new target and wait for it to switch
function TargetBeacon(beacon_new)
    if beacon_desired == beacon_new then
        return
    end
    beacon_desired = beacon_new
    --gcs:send_text(0, string.format("Set desired %s %d old %d", BeaconName(beacon_new), beacon_new, beacon_current))
    now_wait_ms = now_ms

    gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. " State  " .. BeaconName(beacon_current))
    gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. " Target " .. BeaconName(beacon_desired))
end

function ManualSwitch()
    if rc_state_switch == nil then
        return
    end
    local sw_pos = rc_state_switch:get_aux_switch_pos()

    if sw_pos ~= sw_pos_old then
        if beacon_current == BeaconState.Flashing then
            TargetBeacon(BeaconState. Solid)
        elseif beacon_current == BeaconState.Solid then
            TargetBeacon(BeaconState. SlowBlink)
        elseif beacon_current == BeaconState.SlowBlink then
            TargetBeacon(BeaconState.FastBlink)
        elseif beacon_current == BeaconState.FastBlink then
            TargetBeacon(BeaconState.Off)
        elseif beacon_current == BeaconState.Off then
            TargetBeacon(BeaconState.Flashing)
        end
        sw_pos_old = sw_pos
    end
end

local now_debug_ms = now_ms
local beacon_display = -1
function Update()
    now_ms = millis()

    if now_ms > now_debug_ms + 10000 or beacon_current ~= beacon_display then
        -- gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. " State  " .. BeaconName(beacon_current))
        now_debug_ms = now_ms
        beacon_display = beacon_current
    end
    -- Canadian CAR part IX anti-collision lights shall:
    -- * be turned on
    -- * be white in colour;
    -- * flash at a rate of not less than 40, and not more than 100, cycles per minute;

    -- So If the arming state changed the beacon needs to change to flashing/off
    if (arming:is_armed() and not is_armed) then
        is_armed = true
        TargetBeacon(BeaconState.FastBlink)
    elseif (not arming:is_armed() and is_armed) then
        is_armed = false
        TargetBeacon(BeaconState.Off)
    else
        ManualSwitch()
    end

    -- the device cycles through each type, so need to cycle through to get where we are going
    WaitForBeacon()
    OutputChannel()
end

-- wrapper around update(). This calls update() at 1s/REFRESH_RATE Hz
-- and if update faults then an error is displayed, but the script is not
-- stopped
function Protected_Wrapper()
    local success, err = pcall(Update)
    if success then
        return Protected_Wrapper, 1000 / refresh_rate
    end

    gcs:send_text(MAV_SEVERITY.ALERT, SCRIPT_NAME_SHORT .. " Internal Error: " .. err)
    -- when we fault we run the update function again after 1s, slowing it
    -- down a bit so we don't flood the console with errors
    return Protected_Wrapper, 1000
end

function Delayed_Startup()
    gcs:send_text(MAV_SEVERITY.INFO, string.format("%s %s script loaded", SCRIPT_NAME, SCRIPT_VERSION) )
    if rc_state_switch == nil then
        gcs:send_text(MAV_SEVERITY.WARNING, string.format("%s Beacon RC %d not set", SCRIPT_NAME_SHORT, STATE_OPTION) )
    end
    if SRV_Channels:find_channel(SERVO_FUNCTION) == nil then
        gcs:send_text(MAV_SEVERITY.WARNING, string.format("%s Beacon SERVO %d not set", SCRIPT_NAME_SHORT, SERVO_FUNCTION) )
    end

    if rc_on_switch then
        -- gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. " on sw " .. tostring(rc_on_switch:get_aux_switch_pos()))
        if rc_on_switch:get_aux_switch_pos() > 0 then
            gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. " ON ONLY")
            return
        else
            gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. " switchable ")
        end
    end

    -- Default the switch to it's current state
    if rc_state_switch then
        sw_pos_old = rc_state_switch:get_aux_switch_pos()
    end
    OutputChannel()
    TargetBeacon(BeaconState.Off)
    now_wait_ms = now_ms

    return Protected_Wrapper, 1000
end

-- start running update loop - waiting 25s for the AP to initialize if not armed
if FWVersion:type() == 3 then
    if arming:is_armed() then
        is_armed = true
        return Delayed_Startup()
    else
        return Delayed_Startup, 25000
    end
else
    gcs:send_text(MAV_SEVERITY.ERROR, string.format("%s: must run on Plane", SCRIPT_NAME_SHORT))
end
