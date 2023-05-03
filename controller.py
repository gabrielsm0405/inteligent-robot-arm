# Make sure to have the server side running in CoppeliaSim: 
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time
import math
import threading

def moveToConfig(handles,maxVel,maxAccel,targetConf):
    threads = list()
    for i in range(len(handles)):
        currentConf = sim.simxGetJointPosition(clientID, handles[i], sim.simx_opmode_blocking)[1]
        x = threading.Thread(target=movCallback, args=(currentConf, targetConf[i], maxVel[i], maxAccel[i],handles[i], i))
        threads.append(x)
        x.start()
    
    for thread in threads:
        thread.join()

def movCallback(currentConf, config,vel,accel,handle, id):
    interval = 0.001
    diff = 999
    t = 0
    vel = vel * (math.pi/180)
    accel = accel * (math.pi/180)
    while diff >= 0.05 or diff <= -0.05:
        diff = config*(math.pi/180) - currentConf

        deltaS = vel*t + (accel*(t**2))/2
        
        if diff > 0.05:
            currentConf = currentConf+deltaS
            sim.simxSetJointPosition(clientID, handle, currentConf, sim.simx_opmode_blocking)
        elif diff < -0.05:
            currentConf = currentConf-deltaS
            sim.simxSetJointPosition(clientID, handle, currentConf, sim.simx_opmode_blocking)
        t = t + interval
        time.sleep(interval)
    return
        

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')
    
    jointHandles={}
    
    jointHandles[0] = sim.simxGetObjectHandle(clientID, "/joint", sim.simx_opmode_blocking)[1]
    jointHandles[1] = sim.simxGetObjectHandle(clientID, "/joint/joint", sim.simx_opmode_blocking)[1]
    jointHandles[2] = sim.simxGetObjectHandle(clientID, "/joint/joint/joint", sim.simx_opmode_blocking)[1]
    jointHandles[3] = sim.simxGetObjectHandle(clientID, "/joint/joint/joint/joint", sim.simx_opmode_blocking)[1]
    
    vel=360  
    accel=40
    jerk=80
    maxVel=[vel,vel,vel,vel]
    maxAccel=[accel,accel,accel,accel]

    targetPos=[0, 30, 45, 60]
    moveToConfig(jointHandles,maxVel,maxAccel,targetPos)
    targetPos=[0, 30, 45, 0]
    moveToConfig(jointHandles,maxVel,maxAccel,targetPos)
    targetPos=[0, 0, 0, 0]
    moveToConfig(jointHandles,maxVel,maxAccel,targetPos)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')