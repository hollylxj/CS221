import pybullet as p
import pybullet_data
import os
import ctypes
import numpy
import time
import cv2

#Global Variables
#numOfJoints = range(p.getNumJoints(sawyerId))
# jids = [5, 10, 11, 12, 13, 15, 18]
eef_link_id = 7
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
    global Jacobian
    #TODO: Insert assert statement for len of gravity

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(gravity[0],gravity[1],gravity[2])
    p.setTimeStep(timeStep)
    planeId = p.loadURDF("plane.urdf")
    sawyerId = p.loadURDF(urdfFile, useFixedBase=1,flags=2)
    cubeId = p.loadURDF('cube_small.urdf', [-0.8,0,0.03])
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
    p.resetBasePositionAndOrientation(cubeId,[-0.8,0,0.03],[0,0,0,1])

#Checks for collision and returns (hasCollided, reward)
def checkCollision():
    collisions = p.getContactPoints(sawyerId)
    #print("length of collision:",len(collision))
    #print(collisions)

    for collision in collisions:
        #check whether the robot is touching the cube
        if(collision[2] == cubeId):
            #check whether end-effector is touching the cube
            if(collision[3] == eef_link_id):
                print("success!")
                return (True, 1000)
            else:
                print("sawyer body collided with cube!")
                return (True, 1000)
        
        #check whether cube collided with plane (ignore)
        elif(collision[2] == planeId):
            continue
            #print("collision!")
            return (True, -100)
        else:
            print("Unknown collision!")
            print(collision)
            input("Hit enter to continue...")
            return (True, -100)

    #if no collisions
    return (False, 0.0)
    #return (False, -0.01)
    #cur = p.getLinkState(sawyerId, eef_joint_id, physicsClientId)[0]
    #tar = getTargetPosition()
    #dist = sum([(cur[i] - tar[i])**2 for i in range(len(cur))])
    #return (False, -dist)


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

def getTargetJointPosition(xyz_pos):
    target_position = p.getBasePositionAndOrientation(cubeId)
    #target = p.calculateInverseKinematics(sawyerId, eef_joint_id, xyz_pos, (0, 0, 0, 1))
    target = p.calculateInverseKinematics(sawyerId, eef_joint_id, xyz_pos)
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

def moveTo(joint_position):
    loopCount = 0
    THRESH = 0.01
    MAX_ITER = 1500
    #err = THRESH
    #p.setRealTimeSimulation(1,physicsClientId)
    start = time.time()

    joint_position = joint_position[:eef_joint_id]
    #print('joint positions modified by Kuan: ', joint_position)
    
    p.setJointMotorControlArray(sawyerId,
				jointIndices=jids,
                                controlMode=p.POSITION_CONTROL,
                                physicsClientId=physicsClientId,
                                targetPositions=joint_position
                                )
    

    end = time.time()
        #print('time of one step:', end - start )
    p.stepSimulation()

    iter = 0
    while(iter < MAX_ITER):
        #print("error = ",err)
        current_joints = readQ()
        #print("current", current_joints)
        #print("set to", joint_position)
        err = sum(((current_joints[i] - joint_position[i]))**2 for i in range(len(joint_position)))
        if err < THRESH:
            return True
#joint_position = joint_position[:eef_link_id]
        #print('joint positions from IK: ', joint_position)
        #print('current joint positions : ', current_joints)

#p.setJointMotorControlArray(sawyerId,
#                        jointIndices=jids,
#                         controlMode=p.POSITION_CONTROL,
#                         physicsClientId=physicsClientId,
#                          targetPositions=joint_position
#                           )

        p.stepSimulation()
        iter += 1
    return False


#  eef_pos = p.getLinkState(sawyerId, eef_link_id, physicsClientId)[0]


#print("Cur Joints:", current_joints)
#print("Des Joints:", joint_position)
#print("EEF:",eef_link_id, eef_pos)
#print("Error:", (eef_pos[0]-0.3,eef_pos[1]+0.2,eef_pos[2]+0.5))
#EEF: (0.8546297934954054, -0.19670004904818303, 0.48057521767210976)
#Error: (0.05462979349540531, 0.8546297934954054, 0.8546297934954054)

# TODO: Camera rendering
    #img = p.getCameraImage(1000,1000)

