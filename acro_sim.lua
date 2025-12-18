-- Acro FPV drone simulator for OpenTX --
-- Author: Eric Pedley

local thrustWeightRatio = 4.0
local camAngle = 30.0
local camHFOV = 110.0 -- focal length can be calculated with LCD_W

local rcRate = 1.0
local rate=0.7
local expo=0.0

local gravity=9.81

local dronePosition = {x = 0, y = 0, z = 0}
local droneSpeed = {x=0,y=0,z=0}
local droneQuat = {x=0, y=0, z=0, w=0}

local angVelBody = {x=0, y=0, z=0}
local thrust = 0



local function stickToAngularVel(stickInput)
    -- takes stickInput scaled from -1 to 1, and returns angular velocity in degrees/second --
    return 200* (rcRate+(max(0,14.54*(rcRate-2)))) * stickInput * (abs(stickInput)**3 * expo + 1 - expo) / max(1-abs(stickInput)*rate, 0.01)
end

local function projectPoint(x3D, y3D, z3D)
    -- does a pinhole projection to get 2D coords on the LCD screen 

end

local function render()

end

local function game_step()
    angVelBody.x = stickToAngularVel(getValue('ail'))
    angVelBody.y = stickToAngularVel(getValue('ele'))
    angVelBody.z = stickToAngularVel(getValue('yaw'))
    thrust = getValue('thr') * thrustWeightRatio * gravity
end

local function init_func()
end

local function run_func(event)
  if not raceStarted then
    lcd.clear()
    lcd.drawText(LCD_W/2 - 59, 54, "Press [Enter] to start")
    if counter then
      lcd.drawText(LCD_W/2 - 27, 28, "Result:")
      lcd.drawNumber(LCD_W/2 + 12, 28, counter, BOLD)
      if isNewBest then
        lcd.drawText(LCD_W/2 - 42, 2, "New best score!")
      else
        lcd.drawText(LCD_W/2 - 37, 2, "Best score:")
        lcd.drawNumber(LCD_W/2 + 26, 2, bestResult)
      end
    else
      lcd.drawText(LCD_W/2 - 47, 28, "Lua FPV Simulator", BOLD)
    end
    if event == EVT_ENTER_BREAK then
      drone.x = 0
      drone.y = 0
      drone.z = 0
      objectCounter = 0
      for i = 1, objectsN do
        objects[i] = generateObject(zObjectsStep * i)
      end
      counter = 0
      countDown = 3
      startTime = getTime() + countDown * 100
      finishTime = getTime() + (raceTime + countDown) * 100
      countDown = countDown + 1
      startTonePlayed = false
      raceStarted = true
      isNewBest = false
    end
  else
    if lowFps then
      fpsCounter = fpsCounter + 1
      if fpsCounter == 2 then
        fpsCounter = 0
        return 0
      end
    end
    lcd.clear()
    currentTime = getTime()
    if currentTime < startTime then
      local cnt = (startTime - currentTime) / 100 + 1
      if cnt < countDown then
        playTone(1500, 100, 0)
        countDown = countDown - 1
      end
      lcd.drawNumber(LCD_W/2 - 2, LCD_H - LCD_H/3, cnt, BOLD)
    elseif currentTime < finishTime then
      if (currentTime - startTime) < 100 then
        lcd.drawText(LCD_W/2 - 6, 48, 'GO!', BOLD)
        if not startTonePlayed then
          playTone(2250, 500, 0)
          startTonePlayed = true
        end
      end
      if speed.z < 0 then speed.z = 0 end
      drone.y = drone.y - speed.y
      if drone.y >= 0 then
        drone.y = 0
        speed.z = 0
        speed.x = 0
      end
      drone.z = drone.z + speed.z
      drone.x = drone.x + speed.x
      if drone.x > track.w * 3 then drone.x = track.w * 3 end
      if drone.x < -track.w * 3 then drone.x = -track.w * 3 end
      if drone.y < -track.h then drone.y = -track.h end
    else
      if (not bestResult) or (counter > bestResult) then
        isNewBest = true
        saveBestResult(counter)
        bestResult = counter
      end
      raceStarted = false
    end
    remainingTime = (finishTime - currentTime)/100 + 1
    if remainingTime > raceTime then remainingTime = raceTime end
    lcd.drawTimer(LCD_W - 25, 2, remainingTime)
    local closestDist = drone.z + zObjectsStep * objectsN
    for i = 1, objectsN do
      if objects[i].z < closestDist and objects[i].z > (drone.z + speed.z) then
        closestN = i
        closestDist = objects[i].z
      end
    end
    for i = 1, objectsN do
      if drone.z >= objects[i].z then
        success = false
        if objects[i].t == "gateGround" then
          if (math.abs(objects[i].x - drone.x) <= gate.w/2) and (drone.y > -gate.h) then
            success = true
          end
        elseif objects[i].t == "gateAir" then
          if (math.abs(objects[i].x - drone.x) <= gate.w/2) and (drone.y < -gate.h) and (drone.y > -2*gate.h) then
            success = true
          end
        elseif objects[i].t == "flagLeft" then
          if (objects[i].x < drone.x) and (drone.y > -2*gate.h) then
            success = true
          end
        elseif objects[i].t == "flagRight" then
          if (objects[i].x > drone.x) and (drone.y > -2*gate.h) then
            success = true
          end
        end
        if success then
          counter = counter + 1
          playTone(1000, 100, 0)
        else
          counter = counter - 1
          playTone(500, 300, 0)
        end
        objects[i] = generateObject()
      else
        drawObject(objects[i], i == closestN)
      end
    end
    drawLandscape()
    lcd.drawNumber(3, 2, counter)
    if event == EVT_EXIT_BREAK then
      raceStarted = false
      counter = nil
    end
  end
  return 0
end

return { init=init_func, run=run_func }
