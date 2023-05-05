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

import math
from joblib import load
import numpy as np
import warnings
warnings.filterwarnings('ignore')

identification_model = load('random_forest_identification.joblib')
learning_model = load('learning_model3.joblib')

vel=180
accel=90

def moveToHandle(jointHandles, gripperHandle, targetHandle, precision):
    curTarget = sim.simxGetObjectPosition(clientID, targetHandle, -1, sim.simx_opmode_blocking)[1]
    error = 999
    delta = 0.01

    while error > precision:
        anglesPred = learning_model.predict([curTarget])[0].tolist()
        for i in range(len(jointHandles)):
            currentConf = sim.simxGetJointPosition(clientID, jointHandles[i], sim.simx_opmode_blocking)[1]
            diff = anglesPred[i]*(math.pi/180) - currentConf
            if diff > delta:
                sim.simxSetJointTargetPosition(clientID, jointHandles[i], currentConf+delta, sim.simx_opmode_oneshot)
            elif diff < delta:
                sim.simxSetJointTargetPosition(clientID, jointHandles[i], currentConf-delta, sim.simx_opmode_oneshot)
        
        output = sim.simxGetObjectPosition(clientID, gripperHandle, -1, sim.simx_opmode_blocking)[1]
        pred_output = identification_model.predict([anglesPred])[0].tolist()
        feedback = np.subtract(output, pred_output).tolist()
        reference = sim.simxGetObjectPosition(clientID, targetHandle, -1, sim.simx_opmode_blocking)[1]

        curTarget = np.subtract(reference, feedback).tolist()

        error = np.linalg.norm(np.subtract(output, reference))

def moveToPosition(jointHandles, gripperHandle, targetPos, precision):
    curTarget = targetPos
    error = 999
    delta = 0.02
    while error > precision:
        anglesPred = learning_model.predict([curTarget])[0].tolist()
        for i in range(len(jointHandles)):
            currentConf = sim.simxGetJointPosition(clientID, jointHandles[i], sim.simx_opmode_blocking)[1]
            diff = anglesPred[i]*(math.pi/180) - currentConf
            if diff > 0:
                sim.simxSetJointTargetPosition(clientID, jointHandles[i], currentConf+delta, sim.simx_opmode_oneshot)
            else:
                sim.simxSetJointTargetPosition(clientID, jointHandles[i], currentConf-delta, sim.simx_opmode_oneshot)
        
        output = sim.simxGetObjectPosition(clientID, gripperHandle, -1, sim.simx_opmode_blocking)[1]
        pred_output = identification_model.predict([anglesPred])[0].tolist()
        feedback = np.subtract(output, pred_output).tolist()
        reference = targetPos

        curTarget = np.subtract(reference, feedback).tolist()

        error = np.linalg.norm(np.subtract(output, reference))
        

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

    gripperHandle = sim.simxGetObjectHandle(clientID, "/PhantomXPincher/gripperCenter_joint", sim.simx_opmode_blocking)[1]
    
    cuboidHandle = sim.simxGetObjectHandle(clientID, "/Cuboid", sim.simx_opmode_blocking)[1]
    discHandle = sim.simxGetObjectHandle(clientID, "/Disc", sim.simx_opmode_blocking)[1]

    sim.simxSetInt32Signal(clientID, 'PhantomXPincher__15___gripperClose', 0, sim.simx_opmode_blocking)

    moveToHandle(
        jointHandles,
        gripperHandle,
        cuboidHandle,
        0.027
    )

    sim.simxSetInt32Signal(clientID, 'PhantomXPincher__15___gripperClose', 1, sim.simx_opmode_blocking)

    moveToPosition(
        jointHandles,
        gripperHandle,
        [-0.375, 0.075, 0.2],
        0.09
    )

    sim.simxSetInt32Signal(clientID, 'PhantomXPincher__15___gripperClose', 0, sim.simx_opmode_blocking)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')