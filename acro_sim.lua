-- Acro FPV drone simulator for OpenTX --
-- Author: Eric Pedley

-- Cache math functions for performance
local sqrt = math.sqrt
local sin = math.sin
local cos = math.cos
local abs = math.abs
local max = math.max
local min = math.min
local tan = math.tan
local floor = math.floor

-- Pre-computed math constants
local PI_OVER_180 = math.pi / 180

local settingsPath = "/SCRIPTS/acro_settings.txt"

local topSpeed = 20 -- meters per second
local thrustWeightRatio = 4.0
local camAngle = 25.0 -- degrees, positive = tilted up
local camHFOV = 120.0 -- horizontal field of view in degrees

local rcRate = 1.0
local rate = 0.7
local expo = 0.0

local gravity = 9.81

-- Settings menu state
local settingNames = {"Top Speed", "TWR", "Cam Angle"}
local currentSettingIndex = 1  -- 1=topSpeed, 2=TWR, 3=camAngle
local settingsDisplayTime = 0  -- time when settings were last changed
local settingsDisplayTimeout = 100  -- 1 second (in 10ms units)
local lastEvent = 0  -- for debugging button events

------ SETTINGS PERSISTENCE ------

local function loadSettings()
  local f = io.open(settingsPath, "r")
  if f == nil then
    return
  end
  local content = io.read(f, 100)
  io.close(f)
  if content then
    local ts, twr, ca = string.match(content, "([%d%.]+),([%d%.]+),([%d%.]+)")
    if ts then topSpeed = tonumber(ts) end
    if twr then thrustWeightRatio = tonumber(twr) end
    if ca then camAngle = tonumber(ca) end
  end
end

local function saveSettings()
  local f = io.open(settingsPath, "w")
  if f then
    io.write(f, string.format("%.1f,%.2f,%.1f", topSpeed, thrustWeightRatio, camAngle))
    io.close(f)
  end
end

local function recalculateDragCoeff()
  local maxHorizontalThrust = gravity * sqrt(thrustWeightRatio * thrustWeightRatio - 1)
  dragCoeff = maxHorizontalThrust / (topSpeed * topSpeed)
end

-- Update pre-computed constants (call when settings change or on init)
local function updateConstants()
  camAngleRad = camAngle * PI_OVER_180
  recalculateDragCoeff()
end

-- Initialize LCD-dependent constants (must be called after LCD_W/LCD_H are available)
local function initLcdConstants()
  halfLcdW = LCD_W / 2
  halfLcdH = LCD_H / 2
  focalLength = halfLcdW / tan(camHFOV * PI_OVER_180 / 2)
end

local function getCurrentSettingValue()
  if currentSettingIndex == 1 then
    return topSpeed
  elseif currentSettingIndex == 2 then
    return thrustWeightRatio
  else
    return camAngle
  end
end

local function getCurrentSettingFormat()
  if currentSettingIndex == 1 then
    return string.format("%.0f m/s", topSpeed)
  elseif currentSettingIndex == 2 then
    return string.format("%.2f", thrustWeightRatio)
  else
    return string.format("%.0f deg", camAngle)
  end
end

local function adjustCurrentSetting(direction)
  if currentSettingIndex == 1 then
    -- Top speed: 10-50, increment 1
    topSpeed = topSpeed + direction * 1
    if topSpeed < 10 then topSpeed = 10 end
    if topSpeed > 50 then topSpeed = 50 end
  elseif currentSettingIndex == 2 then
    -- TWR: 2.0-6.0, increment 0.25
    thrustWeightRatio = thrustWeightRatio + direction * 0.25
    if thrustWeightRatio < 2.0 then thrustWeightRatio = 2.0 end
    if thrustWeightRatio > 6.0 then thrustWeightRatio = 6.0 end
  else
    -- Cam angle: 10-50, increment 5
    camAngle = camAngle + direction * 5
    if camAngle < 10 then camAngle = 10 end
    if camAngle > 50 then camAngle = 50 end
  end
  updateConstants()  -- Update all pre-computed values
  saveSettings()
end

local function cycleSettingIndex()
  currentSettingIndex = currentSettingIndex + 1
  if currentSettingIndex > 3 then
    currentSettingIndex = 1
  end
end

-- Drag coefficient calculated so that at max horizontal flight, drone reaches topSpeed
-- At max horizontal speed, drone tilts so vertical thrust = gravity:
--   TWR * g * sin(theta) = g  =>  sin(theta) = 1/TWR
--   cos(theta) = sqrt(1 - 1/TWR^2) = sqrt(TWR^2 - 1) / TWR
-- Horizontal thrust component: TWR * g * cos(theta) = g * sqrt(TWR^2 - 1)
-- At terminal velocity: drag = horizontal thrust
--   dragCoeff * topSpeed^2 = g * sqrt(TWR^2 - 1)
-- Therefore: dragCoeff = g * sqrt(TWR^2 - 1) / topSpeed^2
local maxHorizontalThrust = gravity * sqrt(thrustWeightRatio * thrustWeightRatio - 1)
local dragCoeff = maxHorizontalThrust / (topSpeed * topSpeed)
local dt = 0.05 -- timestep in seconds (approx 20 fps)

local dronePosition = {x = 0, y = 0, z = 2} -- z is up
local droneSpeed = {x = 0, y = 0, z = 0}
-- quaternion: w is scalar, x/y/z are vector components
-- identity = no rotation, drone facing +X direction
local droneQuat = {w = 1, x = 0, y = 0, z = 0}

-- Cached camera quaternion (updated once per frame)
local cachedCamQuat = nil

-- Pre-computed constants (updated when settings change)
local camAngleRad = 0
local focalLength = 0
local halfLcdW = 0
local halfLcdH = 0
local lcdInitialized = false  -- flag to track LCD constant initialization

-- HUD text caching
local cachedAltText = ""
local cachedAltInt = -9999
local cachedSpdText = ""
local cachedSpdInt = -9999

local lastTime = 0

-- Hard-coded cubes: {x, y, z, size} where z is up
local cubes = {
  {6, 0, 2, 3},
  {-3, -5, 5, 5},
  -- {10, 5, 0.5, 1},
  -- {0, 10, 2, 2},
}

-- Grid settings
local gridSize = 100  -- total grid extent
local gridSpacing = 25 -- spacing between grid lines

------ MATH HELPERS ------

local function clamp(val, minVal, maxVal)
  return max(minVal, min(maxVal, val))
end

local function rad(deg)
  return deg * PI_OVER_180
end

local function deg(r)
  return r / PI_OVER_180
end

------ QUATERNION MATH ------

local function quatNormalize(q)
  local mag = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z)
  if mag < 0.0001 then mag = 1 end
  return {w = q.w/mag, x = q.x/mag, y = q.y/mag, z = q.z/mag}
end

local function quatMultiply(a, b)
  return {
    w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
    y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
    z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
  }
end

local function quatConjugate(q)
  return {w = q.w, x = -q.x, y = -q.y, z = -q.z}
end

-- Rotate a vector by a quaternion
local function quatRotateVec(q, v)
  local qv = {w = 0, x = v.x, y = v.y, z = v.z}
  local result = quatMultiply(quatMultiply(q, qv), quatConjugate(q))
  return {x = result.x, y = result.y, z = result.z}
end

-- Create quaternion from axis-angle (axis should be unit vector, angle in radians)
local function quatFromAxisAngle(ax, ay, az, angle)
  local halfAngle = angle / 2
  local s = sin(halfAngle)
  return {
    w = cos(halfAngle),
    x = ax * s,
    y = ay * s,
    z = az * s
  }
end

-- Update quaternion with body angular velocity (roll=x, pitch=y, yaw=z) in rad/s
local function quatIntegrateBodyRates(q, wx, wy, wz, deltaT)
  -- Angular velocity magnitude
  local wMag = sqrt(wx*wx + wy*wy + wz*wz)
  if wMag < 0.0001 then
    return q
  end
  -- Create rotation quaternion for this timestep
  local angle = wMag * deltaT
  local dq = quatFromAxisAngle(wx/wMag, wy/wMag, wz/wMag, angle)
  -- Apply rotation: new_q = q * dq (body frame rotation)
  return quatNormalize(quatMultiply(q, dq))
end

------ BETAFLIGHT RATES ------

local function stickToAngularVel(stickInput)
  -- Betaflight rates formula, returns rad/s
  local absStick = abs(stickInput)
  local superRate = rate
  local rcCommand = stickInput
  local angleRate = 200 * (rcRate + max(0.0, 14.54*(rcRate-2))) * rcCommand
  local superFactor = 1.0 / max(1.0 - absStick * superRate, 0.01)
  local expoFactor = absStick * absStick * absStick * expo + 1 - expo
  return rad(angleRate * superFactor * expoFactor)
end

------ PROJECTION ------

local nearPlane = 0.1 -- near clipping plane distance

-- Update cached camera quaternion (call once per frame)
local function updateCameraQuat()
  local camQuat = quatConjugate(droneQuat)
  local camTiltQuat = quatFromAxisAngle(0, 1, 0, camAngleRad)
  cachedCamQuat = quatMultiply(camTiltQuat, camQuat)
end

-- Get camera-space coordinates for a world point
local function worldToCameraSpace(worldX, worldY, worldZ)
  -- Transform world point to drone-relative coordinates
  local relX = worldX - dronePosition.x
  local relY = worldY - dronePosition.y
  local relZ = worldZ - dronePosition.z
  
  -- Use cached camera quaternion
  return quatRotateVec(cachedCamQuat, {x = relX, y = relY, z = relZ})
end

-- Project a camera-space point to screen coordinates
local function projectCameraPoint(camPos)
  -- Guard against division by zero or very small values
  -- Use a small epsilon instead of nearPlane to handle clipped points
  if camPos.x < 0.001 then
    return nil
  end
  
  -- Calculate focal length from HFOV
  local focalLength = (LCD_W / 2) / math.tan(rad(camHFOV / 2))
  
  -- Project to screen (flip Y for screen coords, Z becomes screen Y)
  local screenX = LCD_W / 2 - (camPos.y / camPos.x) * focalLength
  local screenY = LCD_H / 2 - (camPos.z / camPos.x) * focalLength
  
  return {x = screenX, y = screenY, depth = camPos.x}
end

-- Clip a line segment against the near plane
-- Returns clipped camera-space endpoints, or nil if entirely behind
local function clipLineToNearPlane(c1, c2)
  local d1 = c1.x - nearPlane
  local d2 = c2.x - nearPlane
  
  -- Both behind near plane
  if d1 < 0 and d2 < 0 then
    return nil, nil
  end
  
  -- Both in front of near plane
  if d1 >= 0 and d2 >= 0 then
    return c1, c2
  end
  
  -- One point behind, one in front - clip the line
  local t = d1 / (d1 - d2)
  local clippedPoint = {
    x = c1.x + t * (c2.x - c1.x),
    y = c1.y + t * (c2.y - c1.y),
    z = c1.z + t * (c2.z - c1.z)
  }
  
  if d1 < 0 then
    -- c1 is behind, c2 is in front
    return clippedPoint, c2
  else
    -- c1 is in front, c2 is behind
    return c1, clippedPoint
  end
end

-- Clip a 2D line segment to the screen bounds using Liang-Barsky algorithm
-- Returns clipped screen coordinates, or nil if entirely outside
local function clipLineToScreen(x1, y1, x2, y2)
  local xmin, ymin, xmax, ymax = 0, 0, LCD_W - 1, LCD_H - 1
  local dx = x2 - x1
  local dy = y2 - y1
  local p = {-dx, dx, -dy, dy}
  local q = {x1 - xmin, xmax - x1, y1 - ymin, ymax - y1}
  local t0, t1 = 0, 1
  
  for i = 1, 4 do
    if p[i] == 0 then
      -- Line is parallel to this edge
      if q[i] < 0 then
        return nil -- Line is outside and parallel
      end
    else
      local t = q[i] / p[i]
      if p[i] < 0 then
        -- Entering edge
        if t > t1 then return nil end
        if t > t0 then t0 = t end
      else
        -- Leaving edge
        if t < t0 then return nil end
        if t < t1 then t1 = t end
      end
    end
  end
  
  local cx1 = x1 + t0 * dx
  local cy1 = y1 + t0 * dy
  local cx2 = x1 + t1 * dx
  local cy2 = y1 + t1 * dy
  
  return cx1, cy1, cx2, cy2
end

------ RENDERING ------

local function drawLine3D(x1, y1, z1, x2, y2, z2)
  -- Transform to camera space
  local c1 = worldToCameraSpace(x1, y1, z1)
  local c2 = worldToCameraSpace(x2, y2, z2)
  
  -- Clip against near plane
  local clipped1, clipped2 = clipLineToNearPlane(c1, c2)
  if not clipped1 or not clipped2 then
    return -- line entirely behind camera
  end
  
  -- Project to screen
  local p1 = projectCameraPoint(clipped1)
  local p2 = projectCameraPoint(clipped2)
  
  -- Guard against failed projection
  if not p1 or not p2 then
    return
  end
  
  -- Clip to screen bounds (handles lines with endpoints outside viewport)
  local sx1, sy1, sx2, sy2 = clipLineToScreen(p1.x, p1.y, p2.x, p2.y)
  if not sx1 then
    return -- line entirely outside screen
  end
  
  lcd.drawLine(sx1, sy1, sx2, sy2, SOLID, FORCE)
end

local function drawCube(cx, cy, cz, size)
  local s = size / 2
  local draw = drawLine3D  -- local reference for speed
  -- 8 corners of the cube
  local corners = {
    {cx - s, cy - s, cz - s},
    {cx + s, cy - s, cz - s},
    {cx + s, cy + s, cz - s},
    {cx - s, cy + s, cz - s},
    {cx - s, cy - s, cz + s},
    {cx + s, cy - s, cz + s},
    {cx + s, cy + s, cz + s},
    {cx - s, cy + s, cz + s},
  }
  -- 12 edges
  local edges = {
    {1,2}, {2,3}, {3,4}, {4,1}, -- bottom
    {5,6}, {6,7}, {7,8}, {8,5}, -- top
    {1,5}, {2,6}, {3,7}, {4,8}, -- verticals
  }
  for _, edge in ipairs(edges) do
    local c1, c2 = corners[edge[1]], corners[edge[2]]
    drawLine3D(c1[1], c1[2], c1[3], c2[1], c2[2], c2[3])
  end
end

local function drawGroundGrid()
  local z = 0 -- ground plane at z=0
  local halfSize = gridSize / 2
  -- Draw grid lines parallel to X axis
  for y = -halfSize, halfSize, gridSpacing do
    drawLine3D(-halfSize, y, z, halfSize, y, z)
  end
  -- Draw grid lines parallel to Y axis
  for x = -halfSize, halfSize, gridSpacing do
    drawLine3D(x, -halfSize, z, x, halfSize, z)
  end
end

local function render()
  lcd.clear()
  
  -- Initialize LCD constants on first render (LCD_W/LCD_H not available at init)
  if not lcdInitialized then
    initLcdConstants()
    lcdInitialized = true
  end
  
  -- Update cached camera quaternion once per frame
  updateCameraQuat()
  
  -- Draw ground grid
  drawGroundGrid()
  
  -- Draw all cubes
  for _, cube in ipairs(cubes) do
    drawCube(cube[1], cube[2], cube[3], cube[4])
  end
  
  -- Calculate current speed
  local currentSpeed = sqrt(droneSpeed.x^2 + droneSpeed.y^2 + droneSpeed.z^2)
  
  -- Draw HUD info with cached strings (only update when value changes)
  local altInt = floor(dronePosition.z * 10)
  if altInt ~= cachedAltInt then
    cachedAltInt = altInt
    cachedAltText = "Alt:" .. string.format("%.1f", dronePosition.z)
  end
  lcd.drawText(2, 2, cachedAltText, SMLSIZE)
  
  local spdInt = floor(currentSpeed * 10)
  if spdInt ~= cachedSpdInt then
    cachedSpdInt = spdInt
    cachedSpdText = "Spd:" .. string.format("%.1f", currentSpeed)
  end
  lcd.drawText(2, 10, cachedSpdText, SMLSIZE)
  
  -- Debug: display last event
  -- if lastEvent ~= 0 then
  --   lcd.drawText(LCD_W - 50, 2, "EVT:" .. tostring(lastEvent), SMLSIZE)
  -- end
  
  -- Draw settings display if recently changed
  local currentTime = getTime()
  if currentTime - settingsDisplayTime < settingsDisplayTimeout then
    local settingText = settingNames[currentSettingIndex] .. ": " .. getCurrentSettingFormat()
    lcd.drawText(LCD_W/2 - 30, LCD_H - 12, settingText, SMLSIZE)
  end
end

------ PHYSICS ------

local function physics_step(deltaT)
  -- Read stick inputs (-1024 to 1024, normalize to -1 to 1)
  local ailInput = getValue('ail') / 1024
  local eleInput = getValue('ele') / 1024
  local rudInput = -getValue('rud') / 1024
  local thrInput = (getValue('thr') + 1024) / 2048 -- 0 to 1
  
  -- Convert to body angular velocities (rad/s)
  local wx = stickToAngularVel(ailInput)  -- roll
  local wy = stickToAngularVel(eleInput)  -- pitch  
  local wz = stickToAngularVel(rudInput)  -- yaw
  
  -- Update orientation
  droneQuat = quatIntegrateBodyRates(droneQuat, wx, wy, wz, deltaT)
  
  -- Calculate thrust vector in world frame
  -- Thrust points along drone's +Z axis (up in body frame)
  local thrustMag = thrInput * thrustWeightRatio * gravity
  local thrustBody = {x = 0, y = 0, z = thrustMag}
  local thrustWorld = quatRotateVec(droneQuat, thrustBody)
  
  -- Calculate drag force (proportional to v^2, opposing velocity)
  local speed = sqrt(droneSpeed.x^2 + droneSpeed.y^2 + droneSpeed.z^2)
  local dragMag = dragCoeff * speed * speed
  local dragX, dragY, dragZ = 0, 0, 0
  if speed > 0.001 then
    -- Drag opposes velocity direction
    dragX = -dragCoeff * speed * droneSpeed.x
    dragY = -dragCoeff * speed * droneSpeed.y
    dragZ = -dragCoeff * speed * droneSpeed.z
  end
  
  -- Apply forces: thrust + gravity + drag
  local ax = thrustWorld.x + dragX
  local ay = thrustWorld.y + dragY
  local az = thrustWorld.z - gravity + dragZ
  
  -- Integrate velocity
  droneSpeed.x = droneSpeed.x + ax * deltaT
  droneSpeed.y = droneSpeed.y + ay * deltaT
  droneSpeed.z = droneSpeed.z + az * deltaT
  
  -- Integrate position
  dronePosition.x = dronePosition.x + droneSpeed.x * deltaT
  dronePosition.y = dronePosition.y + droneSpeed.y * deltaT
  dronePosition.z = dronePosition.z + droneSpeed.z * deltaT
  
  -- Ground collision
  if dronePosition.z < 0 then
    dronePosition.z = 0
    if droneSpeed.z < 0 then
      droneSpeed.z = 0
    end
  end
end

local function resetDrone()
  dronePosition = {x = 0, y = 0, z = 2}
  droneSpeed = {x = 0, y = 0, z = 0}
  droneQuat = {w = 1, x = 0, y = 0, z = 0}
end

------ MAIN FUNCTIONS ------

local function init_func()
  loadSettings()
  updateConstants()   -- Update all pre-computed values
  lastTime = getTime()
  settingsDisplayTime = 0
  resetDrone()
end

local function run_func(event)
  -- Store event for debug display
  if event ~= 0 then
    lastEvent = event
  end
  
  -- Calculate delta time
  local currentTime = getTime()
  local deltaT = (currentTime - lastTime) / 100 -- getTime() is in 10ms units
  lastTime = currentTime
  
  -- Clamp deltaT to avoid physics explosions
  if deltaT > 0.1 then deltaT = 0.1 end
  if deltaT < 0.001 then deltaT = 0.02 end
  
  -- Check for reset (Enter button)
  if event == EVT_ENTER_BREAK then
    resetDrone()
  end
  
  -- Settings controls: PAGE to cycle, PLUS/MINUS to adjust
  if event == 109 then
    cycleSettingIndex()
    settingsDisplayTime = currentTime
  elseif event == 101 then
    adjustCurrentSetting(1)
    settingsDisplayTime = currentTime
  elseif event == 100 then
    adjustCurrentSetting(-1)
    settingsDisplayTime = currentTime
  end
  
  -- Update physics
  physics_step(deltaT)
  
  -- Render the scene
  render()
  
  -- Exit on EXIT button
  if event == EVT_EXIT_BREAK then
    return 1
  end
  
  return 0
end

return { init=init_func, run=run_func }
