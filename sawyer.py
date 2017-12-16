import pybullet as p
#import pybullet_data
import os
import ctypes
import numpy
import time


#Global Variables
#numOfJoints = range(p.getNumJoints(sawyerId))
# jids = [5, 10, 11, 12, 13, 15, 18]
eef_link_id = 8
eef_joint_id = 7
jids = [0, 1, 2, 3, 4, 5, 6]
sawyerId = None     #Call setup function to load this variable
physicsClientId = None     #Call connect function to load this variable
planeId = None     #Call connect function to load this variable

def connect():
    global physicsClientId
    physicsClientId = p.connect(p.GUI)#or p.DIRECT for non-graphical version

def disconnect():
    p.disconnect()

#gravity must have len of 3
def setup(gravity, timeStep, urdfFile):
    global sawyerId
    global cubeId
    global planeId
    global obstacleId
    global Jacobian
    global cube_init
    global obstacle_init
    #TODO: Insert assert statement for len of gravity
    obstacle_init = [1.1,1.1,0.9]
    cube_init = (-0.7,0.7,0.1)
    #cube_init = [-0.8,0.8,0.25]
    #cube_init = [-0.7,0.7,0.1]
    #p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(gravity[0],gravity[1],gravity[2])
    p.setTimeStep(timeStep)
    planeId = p.loadURDF("plane.urdf")
    sawyerId = p.loadURDF(urdfFile, useFixedBase=1,flags=2)
    cubeId = p.loadURDF('sphere2.urdf', cube_init)
    obstacleId = p.loadURDF('cube.urdf', obstacle_init, useFixedBase=1,flags=2)
    #cubeId = p.loadURDF('wheel.urdf', cube_init)
    N_joints = p.getNumJoints(sawyerId,physicsClientId)
    print("number of joints ",N_joints)
    jointLimits = [[] for _ in range(N_joints)]
    print("jointLimits",jointLimits)
    for i in range(len(jointLimits)):
        info = p.getJointInfo(sawyerId,i,physicsClientId)
        jointLimits[i] = [info[8],info[9]]
        print("joint info",info[0], info[1], info[2])
    print("jointLimits",jointLimits)
    print("sawyerId = ",sawyerId, "cubeId = ",cubeId," planeId = ",planeId)
    return jointLimits

#print(p.getConstraintInfo(cubeId,physicsClientId),"\n")
#print(p.getConstraintState(cubeId,physicsClientId),"\n")

def resetCube():
    p.resetBasePositionAndOrientation(cubeId,cube_init,[0,0,0,1])
#def resetObstacle():
    #p.resetBasePositionAndOrientation(obstacleId,obstacle_init,[0,0,0,1])

#Checks for collision and returns (hasCollided, reward)
def checkCollision():
    collisions = p.getContactPoints(sawyerId)
    #print("length of collision:",len(collision))
    #print(collisions)
    
    cube = p.getBasePositionAndOrientation(cubeId)
    cube = cube[0]
    cube_dist = sum(((cube[i] - cube_init[i]))**2 for i in range(len(cube_init)))
    cur = p.getLinkState(sawyerId, eef_link_id, physicsClientId)[0]
    tar = getTargetPosition()
    dist = sum([(cur[i] - cube_init[i])**2 for i in range(len(cur))])*2
    
    # if hit the cube
    #if cube_dist>0.0001 or dist<0.001:
    if cube_dist>0.0001:
        #print(cube,cube_init,cube_dist)
        return (True, 1000)
    
    for collision in collisions:
        #check whether the robot is touching the cube
        if(collision[1] == sawyerId and collision[2] == cubeId):
            #check whether end-effector is touching the cube
            if(collision[3] == eef_link_id):
                print("success!")
                return (True, 1000)
            else:
                print("sawyer body collided with cube!")
                return (True, 1000)
        
        #check whether sawyer collided with plane
        #elif(collision[1] == sawyerId and collision[2] == planeId):
        else:
            #print("collision!")

                #if dist < 0.1:
                #return (True, 10-dist-100)
            return (True, -dist-100)



#if dist < 0.1:
#return (False, 10-dist)
    return (False, -dist)


def resetPos(val):
    for i in range(len(jids)):
        p.resetJointState(sawyerId, jids[i], targetValue=val[i], targetVelocity=0)

    for _ in range(100):
        p.stepSimulation()

def readQ():
    jointStates = p.getJointStates(sawyerId, jids)
    #print("all", jointStates)
    q = [jointState[0] for jointState in jointStates]
    return q

def getTargetJointPosition(xyz_pos):
    target_position = p.getBasePositionAndOrientation(cubeId)
    #target = p.calculateInverseKinematics(sawyerId, eef_joint_id, xyz_pos, (0, 0, 0, 1))
    target = p.calculateInverseKinematics(sawyerId, eef_link_id, xyz_pos)
    #print(target)
    return target

def getTargetPosition():
    return p.getBasePositionAndOrientation(cubeId)[0]

#(0.8, -0.2, -0.5)
#(-0.5187895397113486, 0.0, 0.2014667025921159, 0.045130853517666984, 0.11337679481863136, 0.5080286760282849, 0.49999565712803945, 0.0)
#(0.2, 0.2, 0.2)
#(-0.18537198502458477, 0.0, -0.24237853308362692, 0.008155493685609428, -0.14166265832650343, 0.5505875681606421, 0.49999565712803945, 0.0)

def getActions(state):
    return ((0.1,0,0,0,0,0,0),(0,0.1,0,0,0,0,0),(0,0,0.1,0,0,0,0),(0,0,0,0.1,0,0,0),
            (0,0,0,0,0.1,0,0),(0,0,0,0,0,0.1,0),(0,0,0,0,0,0,0.1),
            (-0.1,0,0,0,0,0,0),(0,-0.1,0,0,0,0,0),(0,0,-0.1,0,0,0,0),(0,0,0,-0.1,0,0,0),
            (0,0,0,0,-0.1,0,0),(0,0,0,0,0,-0.1,0),(0,0,0,0,0,0,0.1))

debugLinesIds = []

def moveTo(joint_position):
    loopCount = 0
    THRESH = 0.01
    MAX_ITER = 1000
    #err = THRESH
    joint_position = joint_position[:eef_joint_id]
    p.setJointMotorControlArray(sawyerId,
				jointIndices=jids,
                                controlMode=p.POSITION_CONTROL,
                                physicsClientId=physicsClientId,
                                targetPositions=joint_position
                                )
    

    eef_pos_pre = p.getLinkState(sawyerId, eef_link_id, physicsClientId)[0]
    p.stepSimulation()

    iter = 0
    while(iter < MAX_ITER):
        eef_pos_cur = p.getLinkState(sawyerId, eef_link_id, physicsClientId)[0]
        #lineId = p.addUserDebugLine(eef_pos_pre, eef_pos_cur, lineColorRGB=[0.0,0.0,1.0], lifeTime=0)
        #debugLinesIds.append(lineId)
        current_joints = readQ()

        err = sum(((current_joints[i] - joint_position[i]))**2 for i in range(len(joint_position)))
        if err < THRESH:
            return True
        eef_pos_pre = eef_pos_cur

        p.stepSimulation()
        iter += 1
    return False

def clearDebugLines():
    for lineId in debugLinesIds:
        p.removeUserDebugItem(lineId, physicsClientId)

def moveTo_test(joint_position):
    loopCount = 0
    THRESH = 0.01
    MAX_ITER = 1500
    #err = THRESH
    joint_position = joint_position[:eef_joint_id]
    p.setJointMotorControlArray(sawyerId,
                                jointIndices=jids,
                                controlMode=p.POSITION_CONTROL,
                                physicsClientId=physicsClientId,
                                targetPositions=joint_position
                                )
        
                                
    eef_pos_pre = p.getLinkState(sawyerId, eef_link_id, physicsClientId)[0]
    p.stepSimulation()
                                
    iter = 0
    while(iter < MAX_ITER):
        eef_pos_cur = p.getLinkState(sawyerId, eef_link_id, physicsClientId)[0]
        lineId = p.addUserDebugLine(eef_pos_pre, eef_pos_cur, lineColorRGB=[0.0,0.0,1.0], lifeTime=0)
        debugLinesIds.append(lineId)
        current_joints = readQ()
                                
        err = sum(((current_joints[i] - joint_position[i]))**2 for i in range(len(joint_position)))
        if err < THRESH:
                return True
        eef_pos_pre = eef_pos_cur
                                                
        p.stepSimulation()
        iter += 1
    return False

