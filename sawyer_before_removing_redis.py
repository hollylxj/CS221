import pybullet as p
import os
import ctypes

# Redis libraries
#################
import redis
import time
#################

# Redis initialization
##############
robot_name = "sawyer"
PY_JOINT_TORQUES_COMMANDED_KEY = "py::robot::" + robot_name + "::actuators::fgc"
PY_JOINT_ANGLES_KEY         = "py::robot::" + robot_name + "::sensors::q"
PY_JOINT_VELOCITIES_KEY     = "py::robot::" + robot_name + "::sensors::dq"
PY_EE_POSITION_KEY          = "py::robot::" + robot_name + "::tasks::ee_pos"
PY_EE_DES_POSITION_KEY      = "py::robot::" + robot_name + "::tasks::ee_pos_des"
PY_EE_DES_ORIENTATION_KEY   = "py::robot::" + robot_name + "::tasks::ee_ori_des"
PY_Q_READY                  = "py::robot::" + robot_name + "::tasks::q_flag";
PY_TORQUE_READY             = "py::robot::" + robot_name + "::tasks::torque_flag";

r=redis.Redis(host='localhost',port=6379,db=0)
##############

#Global Variables
#numOfJoints = range(p.getNumJoints(sawyerId))
#jids = [1,3,4,5,6,7,8]
jids = [5,10,11,12,13,15,18]
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

def disableMotors():
    p.setJointMotorControlArray(sawyerId, jointIndices=jids,
                            controlMode=p.VELOCITY_CONTROL,
                            physicsClientId=physicsClientId,
                            forces=(0.,) * len(jids) )

#pos and orn must have len of 3
def setDesPosOrn(pos, orn):
    #TODO: Include assert statement for len of pos and orn
    pos = "{} {} {}".format(pos[0], pos[1], pos[2])
    orn = "{} {} {}".format(orn[0], orn[1], orn[2])
    r.set(PY_EE_DES_POSITION_KEY, pos)
    r.set(PY_EE_DES_ORIENTATION_KEY, orn)

def readQdQ():
    jointStates = p.getJointStates(sawyerId, jids)
    while jointStates is None:
        jointStates = p.getJointStates(sawyerId, jids)

    raw_q = [jointState[0] for jointState in jointStates]
    raw_dq = [jointState[1] for jointState in jointStates]
    sum_dq = sum([abs(val) for val in raw_dq])

    q = "{} {} {} {} {} {} {}".format(raw_q[0], raw_q[1], raw_q[2], raw_q[3], raw_q[4], raw_q[5], raw_q[6])
    dq = "{} {} {} {} {} {} {}".format(raw_dq[0], raw_dq[1], raw_dq[2], raw_dq[3], raw_dq[4],raw_dq[5], raw_dq[6])
    print('dq:::',dq)
    return [q, dq, sum_dq]

def sendQdQ(q,dq):
    # Wait for Sawyer Controller to finish reading previous Q Value
    #start = time.time()
    q_isReady = r.get(PY_Q_READY)
    while q_isReady is not None and q_isReady == "Ready to Read":
        q_isReady = r.get(PY_Q_READY)
        #end = time.time()
        #print('waiting for reading previous Q:',end-start)
        
    #Write q and dq values to Redis Client
    r.set(PY_JOINT_ANGLES_KEY, q)
    r.set(PY_JOINT_VELOCITIES_KEY, dq)
    # update pos and orn if changed
    #r.set(PY_EE_DES_POSITION_KEY, pos)
    #r.set(PY_EE_DES_ORIENTATION_KEY, orn)
    r.set(PY_Q_READY, "Ready to Read")

def readTorque():
    #Wait for torque value to be ready
    #start = time.time()
    torque_isReady = r.get(PY_TORQUE_READY)
    while torque_isReady != "Ready to Read":
        torque_isReady = r.get(PY_TORQUE_READY)
    #end = time.time()
    #print('seconds used getting torque:',end-start)

    #Read torque values
    tau = r.get(PY_JOINT_TORQUES_COMMANDED_KEY) # Get torque from PID
        #Tell Saw Controller that torque value has been read
    r.set(PY_TORQUE_READY, "Ready to Write")
        #Build float array
    tau_floats = [float(x) for x in tau.split()]
    return tau_floats

def sendTorque(torque):
    #Apply torques
    p.setJointMotorControlArray(sawyerId, jids,
                                controlMode=p.TORQUE_CONTROL,
                                physicsClientId=physicsClientId,
                                forces=torque)

def moveTo():
    loopCount = 0
    #p.setRealTimeSimulation(1,physicsClientId)
    while(1):
        start = time.time()

        # read Q and dQ values from pybullet simulator
        [q,dq,sum_dq] = readQdQ()
        
        loopCount += 1
        if sum_dq<0.01 and loopCount>10:
            print('done moving to desired position')
            break

        # send Q and dQ values to Redis, which will pass them to sawyer pid controller to calculate next torque
        sendQdQ(q,dq)

        # read torque needs to be applied from redis
        torque=readTorque()
        
        print(torque)
    
        # set pybullet simulator joints to apply torque
        sendTorque(torque)
    
        end = time.time()
        #print('time of one step:', end - start )
        p.stepSimulation()
