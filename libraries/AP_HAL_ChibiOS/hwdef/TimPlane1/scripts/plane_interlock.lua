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

 Arm/Emergenycy Stop Interlock for Plane

 This code will prevent the Arm/Emergency Stop switch from triggering unless the pilot is also holding
 a second switch (interlock) high. This switch should ideally be set to a momentary switch, so the arm/emergency stop
 is only active when the interlock button is being actively pressed.
 
 For this to work correctly there should be a channel set to FUNCTION=165, but not being sent by the transmitter. 
 Yes in other words this uses THREE channels!
--]]

SCRIPT_NAME = "Arm/E-Stop interlock"
SCRIPT_NAME_SHORT = "AEST-Lock"
SCRIPT_VERSION = "4.7.0-003"

REFRESH_RATE = 20   -- Hertz
STARTUP_DELAY = 25  -- wait this many seconds for the FC to come up before starting the script

FLIGHT_MODE = {AUTO=10, RTL=11, LOITER=12, GUIDED=15, QHOVER=18, QLOITER=19, QRTL=21}

MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
MAV_CMD_INT = { DO_SET_MODE = 176, DO_CHANGE_SPEED = 178, DO_REPOSITION = 192, 
                  GUIDED_CHANGE_SPEED = 43000, GUIDED_CHANGE_ALTITUDE = 43001, GUIDED_CHANGE_HEADING = 43002 }
MAV_SPEED_TYPE = { AIRSPEED = 0, GROUNDSPEED = 1, CLIMB_SPEED = 2, DESCENT_SPEED = 3 }
MAV_HEADING_TYPE = { COG = 0, HEADING = 1} -- COG = Course over Ground, i.e. where you want to go, HEADING = which way the vehicle points 

local PARAM_TABLE_KEY = 104
local PARAM_TABLE_PREFIX = "INTLCK_"

-- bind a parameter to a variable
function bind_param(name)
    return Parameter(name)
end

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), SCRIPT_NAME_SHORT .. string.format('could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- setup follow mode specific parameters need 2wo tables because there are > 10 parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 10), SCRIPT_NAME_SHORT .. 'could not add param table: ' .. PARAM_TABLE_PREFIX)

--[[
    // @Param: INTLCK_ACT_FN
    // @DisplayName: Arm/EStop Function
    // @Description: Setting an RC channel's _OPTION to this value will use it for as the main Arm/EStop function (similar to the standard RCx_FUNCTION = 165)
    // @Range: 300 307
--]]
INTLCK_ACT_FN = bind_add_param("ACT_FN", 1, 306)

--[[
    // @Param: INTLCK_LCK_FN
    // @DisplayName: Arm/EStop Interlock Function
    // @Description: Setting an RC channel's _OPTION to this value will use it for the Interlock override
    // @Range: 300 307
--]]
INTLCK_LCK_FN = bind_add_param("LCK_FN", 2, 307)

AESTOP_CHANNEL = rc:find_channel_for_option(165) -- The RC channel used for Arm/Emergency Stop

THROTTLE_CHANNEL_NO = Parameter('RCMAP_THROTTLE'):get() or 3
THROTTLE_CHANNEL = rc:get_channel(Parameter('RCMAP_THROTTLE'):get()) -- The RC channel used for throttle
THROTTLE_CHANNEL_PREFIX = string.format("RC%.0f_", THROTTLE_CHANNEL_NO)
THROTTLE_CHANNEL_MIN = Parameter(THROTTLE_CHANNEL_PREFIX .. "MIN"):get() or 1000
THROTTLE_CHANNEL_MAX = Parameter(THROTTLE_CHANNEL_PREFIX .. "MAX"):get() or 1000
THROTTLE_CHANNEL_TRIM = Parameter(THROTTLE_CHANNEL_PREFIX .. "TRIM"):get() or 1000

local last_switch_state = 0

local estop_pwm = 1000

local function estop_update()
    AESTOP_CHANNEL:set_override(estop_pwm)
end

local function estop_motors()
    estop_pwm = 1050
    gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. " motors OFF")
end

local function enable_motors()
    estop_pwm = 1500
    gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. " motors ON")
end

local function attempt_arm()
    estop_pwm = 1950
    gcs:send_text(MAV_SEVERITY.NOTICE, SCRIPT_NAME_SHORT .. " arming ...")
end

local function update()
    local switch_function = INTLCK_ACT_FN:get()
    if switch_function == nil then
        return
    end
    local switch_state = rc:get_aux_cached(switch_function) or -1
    if (switch_state ~= last_switch_state) then
	-- gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. " switch:"..switch_state)
        -- we have a change on the main switch, but is the pilot holding down the interlock?
        local lock_function = INTLCK_LCK_FN:get()
	if lock_function == nil then
	    gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. "Interlock not set")
	    return
	end
	local lock_state = rc:get_aux_cached(lock_function) or -1
	local throttle_pwm = rc:get_pwm(THROTTLE_CHANNEL_NO)
	local throttle_norm = THROTTLE_CHANNEL:norm_input_dz()

	-- gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. string.format(" pwm: %.0f norm %.0f", throttle_pwm, throttle_norm))

	-- only execute the arm/estop if interlock is held (should be a momentary switch to work best)
	--gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. " lock:"..lock_state)
        if lock_state > 0 then
            if switch_state == 0 then -- switch Low to disarm - so defaults to on
                estop_motors()
            elseif switch_state == 1 then
                enable_motors()
            elseif switch_state == 2 then -- switch High to turn off
                attempt_arm()
            end
        else
	    gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. " no Interlock")
        end
        last_switch_state = switch_state
    end
    -- have to keep applying the update otherwise it will revert for some reason
    estop_update()
end

-- wrapper around update(). This calls update() at REFRESHRATE Hz, i.e. every 1000/REFRESH_RATE milliseconds
-- and if update faults then an error is displayed, but the script is not
-- stopped
function Protected_Wrapper()
    local success, err = pcall(update)
    if not success then
       gcs:send_text(0, SCRIPT_NAME_SHORT .. ": Error: " .. err)
       -- when we fault we run the update function again after 1s, slowing it
       -- down a bit so we don't flood the console with errors
       return Protected_Wrapper, 1000
    end
    return Protected_Wrapper, 1000 / REFRESH_RATE
end

function Delayed_Startup()
    gcs:send_text(MAV_SEVERITY.INFO, string.format("%s %s script loaded", SCRIPT_NAME, SCRIPT_VERSION) )
    return Protected_Wrapper()
end

-- wait a bit for AP to come up then start running update loop
if AESTOP_CHANNEL ~= nil then
    estop_motors()
    return Delayed_Startup, 1000 * STARTUP_DELAY
else
    gcs:send_text(MAV_SEVERITY.NOTICE,string.format("%s: Arm/Emergency Stop not set", SCRIPT_NAME_SHORT))
end
