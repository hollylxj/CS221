import pybullet as p
import os
import time

# Redis libraries
#################
import redis
import time
from math import pi
#################

# Redis initialization
##############
robot_name = "sawyer"
JOINT_TORQUES_COMMANDED_KEY = "cs225a::robot::" + robot_name + "::actuators::fgc"
JOINT_ANGLES_KEY            = "cs225a::robot::" + robot_name + "::sensors::q"
JOINT_VELOCITIES_KEY        = "cs225a::robot::" + robot_name + "::sensors::dq"
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

def setDesPosOrn(pos, orn):
    r.set(PY_EE_DES_POSITION_KEY, pos)
    r.set(PY_EE_DES_ORIENTATION_KEY, orn)
    q_isReady = None

def moveTo(physicsClient,sawyerId):
    while(1):
        start = time.time()
        #print("all joint states:",len(p.getJointStates(sawyerId ,[0,1,2,3,4,5,6,7,8]))) -- 9
        jointStates = p.getJointStates(sawyerId, [1,3,4,5,6,7,8])
        if jointStates is None:
            continue
        raw_q = [jointState[0] for jointState in jointStates]
        raw_dq = [jointState[1] for jointState in jointStates]
        sum_dq = sum([abs(val) for val in raw_dq])
        if sum_dq<0.3:
            print('done moving to desired position')
            break
        q = "{} {} {} {} {} {} {}".format(raw_q[0], raw_q[1], raw_q[2], raw_q[3], raw_q[4], raw_q[5], raw_q[6])
        dq = "{} {} {} {} {} {} {}".format(raw_dq[0], raw_dq[1], raw_dq[2], raw_dq[3], raw_dq[4], raw_dq[5], raw_dq[6])
        print('dq:::',dq)


        # Wait for Sawyer Controller to finish reading previous Q Value
        q_isReady = r.get(PY_Q_READY)
        #start = time.time()
    
    
        while q_isReady is not None and q_isReady == "Ready to Read":
            q_isReady = r.get(PY_Q_READY)
        #end = time.time()
        #print('seconds used waiting for Sawyer Controller to finish reading previous Q Value:',end-start)

        #Write values to Redis Client
        r.set(PY_JOINT_ANGLES_KEY, q)
        r.set(PY_JOINT_VELOCITIES_KEY, dq)
        #r.set(PY_EE_DES_POSITION_KEY, pos)
        r.set(PY_Q_READY, "Ready to Read")
    
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
        #print(tau_floats)
    
    
        #Apply torques
        p.setJointMotorControlArray(sawyerId, [1,3,4,5,6,7,8],
                                controlMode=p.TORQUE_CONTROL,
                                physicsClientId=physicsClient,
                                forces=tau_floats)

        end = time.time()
        #print('time of one step:', end - start )
        p.stepSimulation()
