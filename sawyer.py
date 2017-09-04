import pybullet as p
import os
import ctypes
from SawyerController.SawyerController import SawyerController as SC
import time





#Global Variables
#numOfJoints = range(p.getNumJoints(sawyerId))
jids = [1,3,4,5,6,7,8]
#jids = [5,10,11,12,13,15,18]
sawyerId = None     #Call setup function to load this variable
physicsClientId = None     #Call connect function to load this variable
sc = SC('sawyer.urdf')

# connect pybullet simulator
def connect():
    global physicsClientId
    physicsClientId = p.connect(p.GUI)#or p.DIRECT for non-graphical version

def disconnect():
    p.disconnect()

# disconnect pybullet simulator
def setup(gravity, timeStep, urdfFile):
    global sawyerId
    #TODO: Insert assert statement for len of gravity
    p.setGravity(gravity[0],gravity[1],gravity[2]) #gravity must have len of 3
    p.setTimeStep(timeStep)
    sawyerId = p.loadURDF(urdfFile, useFixedBase = 1)
    for i in range (p.getNumJoints(sawyerId,physicsClientId)):
        print(i, p.getJointInfo(sawyerId,i,physicsClientId)[1])



# reset joint positions to all 0.5, this is called at the beginning to avoid starting from a singularity position
def resetPos(val):
    for jid in jids:
        p.resetJointState(sawyerId, jid, targetValue=val, targetVelocity=0.,
                      physicsClientId=physicsClientId)
    for _ in range(100):
        p.stepSimulation()

# disable all motors to enable torque mode.
def disableMotors():
    p.setJointMotorControlArray(sawyerId, jointIndices=jids,
                            controlMode=p.VELOCITY_CONTROL,
                            physicsClientId=physicsClientId,
                            forces=(0.,) * len(jids) )

# read current q and dq from pybullet simulator
def readQdQ():
    jointStates = p.getJointStates(sawyerId, jids)
    while jointStates is None:
        jointStates = p.getJointStates(sawyerId, jids)

    q = [jointState[0] for jointState in jointStates]
    dq = [jointState[1] for jointState in jointStates]
    sum_dq = sum([abs(val) for val in dq])

    print('dq:::',dq)
    return [q, dq, sum_dq]

# apply torques to pybullet simulator
def sendTorque(torque):
    #Apply torques
    p.setJointMotorControlArray(sawyerId, jids,
                                controlMode=p.TORQUE_CONTROL,
                                physicsClientId=physicsClientId,
                                forces=torque)
# ask sawyer to move to a given position and orientation
def moveTo(pos,orn):
    loopCount = 0
    #p.setRealTimeSimulation(1,physicsClientId)
    while(1):
        
        start = time.time()
        # read Q and dQ values from pybullet simulator
        [q,dq,sum_dq] = readQdQ()
        
        # if sawyer is moving very slow, stop it.
        loopCount += 1
        if loopCount>10000:
            break
        if sum_dq<0.03 and loopCount>10:
            print('done moving to desired position')
            break


        readQ = time.time()
        # Call SawyerController to calculate torque
        torque= sc.calcTorque(q, dq, pos, orn)
        calcT = time.time()
        


        # set pybullet simulator joints to apply torque
        sendTorque(torque)
    
        sendT = time.time()
        p.stepSimulation()
        print('readQ:', readQ - start, '    calcT:',calcT-readQ,'   sendT:',sendT-calcT)
