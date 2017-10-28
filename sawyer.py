import pybullet as p
import os
import ctypes
import numpy
import time

#Global Variables
#numOfJoints = range(p.getNumJoints(sawyerId))
jids = [1,3,4,5,6,7,8]
#jids = [5,10,11,12,13,15,18]
sawyerId = None     #Call setup function to load this variable
physicsClientId = None     #Call connect function to load this variable

def connect():
    global physicsClientId
    physicsClientId = p.connect(p.GUI)#or p.DIRECT for non-graphical version

def disconnect():
    p.disconnect()

#gravity must have len of 3
def setup(gravity, timeStep, urdfFile):
    global sawyerId
    #TODO: Insert assert statement for len of gravity
    p.setGravity(gravity[0],gravity[1],gravity[2])
    p.setTimeStep(timeStep)
    sawyerId = p.loadURDF(urdfFile, useFixedBase = 1)
    for i in range (p.getNumJoints(sawyerId,physicsClientId)):
        print(i, p.getJointInfo(sawyerId,i,physicsClientId)[1])




def resetPos(val):
    for jid in jids:
        p.resetJointState(sawyerId, jid, targetValue=val, targetVelocity=0.,
                      physicsClientId=physicsClientId)

    for _ in range(100):
        p.stepSimulation()

def readQ():
    jointStates = p.getJointStates(sawyerId, jids)
    #print("all", jointStates)
    q = [jointState[0] for jointState in jointStates]
    return q




def moveTo(joint_position):
    loopCount = 0
    thresh = 0.01
    err = thresh
    #p.setRealTimeSimulation(1,physicsClientId)
    start = time.time()
    
    p.setJointMotorControlArray(sawyerId, jointIndices=jids,
                              controlMode=p.POSITION_CONTROL,
                              physicsClientId=physicsClientId,
                              targetPositions=joint_position )
    

    end = time.time()
        #print('time of one step:', end - start )
    p.stepSimulation()

    while(err>=thresh):
        current_joints = readQ()
        print("current", current_joints)
        print("set to", joint_position)
        err = sum(((current_joints[i] - joint_position[i]))**2 for i in range(len(joint_position)))
        p.stepSimulation()

