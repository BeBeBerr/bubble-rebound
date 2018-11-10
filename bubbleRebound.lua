
function sysCall_threadmain()
    
    bubbleRobBase = sim.getObjectAssociatedWithScript(sim.handle_self) -- bubbleRob's handle
    leftMotor = sim.getObjectHandle("bubbleRob_leftMotor") -- handle of the left motor
    rightMotor = sim.getObjectHandle("bubbleRob_rightMotor") -- handle of the right motor
    
    frontLeftSensor = sim.getObjectHandle("bubbleRob_sensingNose_frontLeft")
    frontRightSensor = sim.getObjectHandle("bubbleRob_sensingNose_frontRight")
    leftSensor = sim.getObjectHandle("bubbleRob_sensingNose_left")
    rightSensor = sim.getObjectHandle("bubbleRob_sensingNose_right")
    midLeftSensor = sim.getObjectHandle("bubbleRob_sensingNose_midLeft")
    midRightSensor = sim.getObjectHandle("bubbleRob_sensingNose_midRight")
    
    -- main loop
    setVelocity(1, 0)
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop 
    do
        
        resultFrontLeft, distanceFrontLeft = sim.readProximitySensor(frontLeftSensor)
        resultFrontRight, distanceFrontRight = sim.readProximitySensor(frontRightSensor)
        resultLeft, distanceLeft = sim.readProximitySensor(leftSensor)
        resultRight, distanceRight = sim.readProximitySensor(rightSensor)
        resultMidLeft, distanceMidLeft = sim.readProximitySensor(midLeftSensor)
        resultMidRight, distanceMidRight = sim.readProximitySensor(midRightSensor)

        readings = {}
        readings[0] = distanceLeft
        readings[1] = distanceMidLeft
        readings[2] = distanceFrontLeft
        readings[3] = distanceFrontRight
        readings[4] = distanceMidRight
        readings[5] = distanceRight

        for i = 0,5 do
            if (readings[i] == nil) -- didn't detect obstacle
            then readings[i] = 0.20
            end
        end

        

        if (checkForObstacles(readings)) 
        then 
            local angle = getReboundAngle(readings)
            print(angle)
            turn(angle)
        end


        sim.switchThread()
    end
end

function checkForObstacles(readings)
    
    bubbleBoundary = {}
    bubbleBoundary[0] = 0.1 -- left sensor
    bubbleBoundary[1] = 0.1
    bubbleBoundary[2] = 0.15
    bubbleBoundary[3] = 0.15
    bubbleBoundary[4] = 0.1
    bubbleBoundary[5] = 0.1 -- right sensor

    for i = 0,5 do
        if ((readings[i] ~= nil) and (readings[i] < bubbleBoundary[i]))
        then
            print("obstacle detected.")
            return true
        end
    end
    
    return false

end


function getReboundAngle(readings)

    alpha0 = 180.0 / 6.0 -- 30 degree
    alpha = {}
    alpha[0] = 3.0 * alpha0; -- left sensor angle
    alpha[1] = 2.0 * alpha0;
    alpha[2] = 1.0 * alpha0;
    alpha[3] = -1.0 * alpha0;
    alpha[4] = -2.0 * alpha0;
    alpha[5] = -3.0 * alpha0; -- right sensor angle

    sum1 = 0.0
    sum2 = 0.0
  
    for i = 0,5 do
        sum1 = sum1 + alpha[i] * readings[i]
        sum2 = sum2 + readings[i]
    end
 
    alpha_R = sum1 / sum2

    return alpha_R


end


function setVelocity(linear, angular) 
    
    rightWheelSpeed = linear
    leftWheelSpeed = linear
    
    if (angular >= 0) 
    then 
        -- turn left
        rightWheelSpeed = rightWheelSpeed + angular
    else
        -- turn right
        leftWheelSpeed = leftWheelSpeed - angular

    end

    sim.setJointTargetVelocity(leftMotor, leftWheelSpeed)
    sim.setJointTargetVelocity(rightMotor, rightWheelSpeed)

end


function turn(angle)

    lastAngle = sim.getObjectOrientation(bubbleRobBase, -1) -- get absoulute rotation angles
    sumAngle = 0.0
 
    while (true)
    do

        currentAngle = sim.getObjectOrientation(bubbleRobBase, -1)
        deltaAngle = math.abs(currentAngle[3] - lastAngle[3])
        if (deltaAngle > 3)
        then -- discontinuity
            deltaAngle = math.abs(currentAngle[3] - lastAngle[3]) - 6.283
        end
        lastAngle = currentAngle
    
        sumAngle = sumAngle + deltaAngle
        if (angle >= 0) 
        then
            setVelocity(0, 1)
        else 
            setVelocity(0, -1)
        end
        
        if ((sumAngle / 3.1415) * 180 >= math.abs(angle))
        then 
            print("finished turning.")
            setVelocity(0.0, 0.0) -- stop
            break
        end


        --sim.switchThread()
    end


end

function sysCall_cleanup()
	-- clean up
end