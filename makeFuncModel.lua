csvPath = "C:/Users/Gabriel Marques/Desktop/inteligent-robot-arm/funcModel.csv"

function sysCall_init()
    centerJoint=sim.getObject('./gripperCenter_joint')
    closeJoint=sim.getObject('./gripperClose_joint')
    corout=coroutine.create(coroutineMain)
end

function sysCall_actuation()
    if coroutine.status(corout)~='dead' then
        local ok,errorMsg=coroutine.resume(corout)
        if errorMsg then
            error(debug.traceback(corout,errorMsg),2)
        end
    end
end

function createCsv(path)
    local csv=io.open(path, "r")
    
    if csv == nil then
        csv=io.open(path, "w")
        csv:write('a1'..","..'a2'..","..'a3'..","..'a4'..","..'gx'..","..'gy'..","..'gz')
        csv:write('\n')
    end
    
    io.close(csv) 
end

function addToCsv(path, data)
    csv=io.open(path, "a")
    
    csv:write(data.a1..","..data.a2..","..data.a3..","..data.a4..","..data.gx..","..data.gy..","..data.gz)
    csv:write('\n')
    
    io.close(csv) 
end

-- This is a threaded script, and is just an example!

function movCallback(config,vel,accel,handles)    
    local servoPositions = {}
    for i=1,#handles,1 do
        if sim.isDynamicallyEnabled(handles[i]) then
            sim.setJointTargetPosition(handles[i],config[i])
        else    
            sim.setJointPosition(handles[i],config[i])
        end
        servoPositions[i] = sim.getJointPosition(handles[i])/(math.pi/180)
    end
    
    gripperPos = sim.getObjectPosition(centerJoint,sim.handle_world)
    addToCsv(csvPath, {
        a1 = servoPositions[1],
        a2 = servoPositions[2],
        a3 = servoPositions[3],
        a4 = servoPositions[4],
        gx = gripperPos[1],
        gy = gripperPos[2],
        gz = gripperPos[3]
    })
    
    --print(
    --    {
    --    servoPositions[1],
    --    servoPositions[2],
    --    servoPositions[3],
    --    servoPositions[4],
    --    sim.getObjectPosition(centerJoint,sim.handle_world),
    --    }
    --)
end

function moveToConfig(handles,maxVel,maxAccel,maxJerk,targetConf)
    local currentConf={}
    for i=1,#handles,1 do
        currentConf[i]=sim.getJointPosition(handles[i])
    end
    sim.moveToConfig(-1,currentConf,nil,nil,maxVel,maxAccel,maxJerk,targetConf,nil,movCallback,handles)
end

function coroutineMain()
    local jointHandles={}
    
    createCsv(csvPath)
    
    for i=1,4,1 do
        jointHandles[i]=sim.getObject('./joint',{index=i-1})
    end

    local modelBase=sim.getObject('.')
    local modelBaseName=sim.getObjectAlias(modelBase,4)
    
    local vel=180  
    local accel=40
    local jerk=80
    local maxVel={vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180}
    local maxAccel={accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180}
    local maxJerk={jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180}

    -- Close the gripper:
    sim.setInt32Signal(modelBaseName..'_gripperClose',1)

    sim.wait(1)
    
    angles = {0, 30, 45, 60, 90, 180, -30, -45, -60, -90, -180}
    
    for a1=1,#angles,1 do
       for a2=1,#angles,1 do
            for a3=1,#angles,1 do
                if  math.abs(angles[a2]) < math.abs(angles[a3]) then
                    for a4=1,#angles,1 do
                        if math.abs(angles[a3]) < math.abs(angles[a4]) then
                                local targetPos={
                                angles[a1]*math.pi/180,
                                angles[a2]*math.pi/180,
                                angles[a3]*math.pi/180,
                                angles[a4]*math.pi/180
                            }
                            moveToConfig(jointHandles,maxVel,maxAccel,maxJerk,targetPos)
                        end
                    end
                end
            end
        end 
    end
    
    -- Open the gripper:
    -- sim.setInt32Signal(modelBaseName..'_gripperClose',0)
end

function sysCall_joint(inData)
    if inData.handle==centerJoint then
        local av=sim.getJointPosition(closeJoint)
        local error=(av/2)-inData.pos
        local ctrl=error*20
        
        local maxVelocity=ctrl
        if (maxVelocity>inData.maxVel) then
            maxVelocity=inData.maxVel
        end
        if (maxVelocity<-inData.maxVel) then
            maxVelocity=-inData.maxVel
        end
        local forceOrTorqueToApply=inData.maxForce

        local outData={vel=maxVelocity,force=forceOrTorqueToApply}
        return outData
    end
end