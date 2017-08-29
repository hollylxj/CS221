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


## Setup Sawyer simulator
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setGravity(0,0,-9.8)
p.setTimeStep(0.0002)
os.chdir("/Users/holly/sawyer.git/bin/resources/sawyer")
sawyerId = p.loadURDF("sawyer.urdf", useFixedBase = 1)
#p.setRealTimeSimulation(1,sawyerId)


## Reseting initial position
jids = list(range(p.getNumJoints(sawyerId)))

for jid in [1,3,4,5,6,7,8]:
    p.resetJointState(sawyerId, jid, targetValue=0.5, targetVelocity=0.,
                  physicsClientId=physicsClient)


count = 0
while(count<=1000):
    p.stepSimulation()
    count +=1
#time.sleep(1)

jointStates = p.getJointStates(sawyerId, [1,3,4,5,6,7,8])

while jointStates is None:
    jointStates = p.getJointStates(sawyerId, [1,3,4,5,6,7,8])

raw_q = [jointState[0] for jointState in jointStates]
raw_dq = [jointState[1] for jointState in jointStates]
q = "{} {} {} {} {} {} {}".format(raw_q[0], raw_q[1], raw_q[2], raw_q[3], raw_q[4], raw_q[5], raw_q[6])
dq = "{} {} {} {} {} {} {}".format(raw_dq[0], raw_dq[1], raw_dq[2], raw_dq[3], raw_dq[4], raw_dq[5], raw_dq[6])
print('q:::',q)

raw_input( "enter to proceed")
#Disable joint motors
p.setJointMotorControlArray(sawyerId, jointIndices=jids,
                            controlMode=p.VELOCITY_CONTROL,
                            physicsClientId=physicsClient,
                            forces=(0.,) * len(jids) )

## set Torque, initial Q and dQ
r.set(PY_JOINT_TORQUES_COMMANDED_KEY,"{} {} {} {} {} {} {}".format(0,0,0,0,0,0,0))
r.set(PY_JOINT_ANGLES_KEY, q)
r.set(PY_JOINT_VELOCITIES_KEY, dq)

pos = "{} {} {}".format(0.0, 0.8, 0.0)
orn = "{} {} {}".format(0.1, 0.1, 0.1)
r.set(PY_EE_DES_POSITION_KEY, pos)
r.set(PY_EE_DES_ORIENTATION_KEY, orn)

q_isReady = None

### Run sawyer.cpp


while(1):
    #print("all joint states:",len(p.getJointStates(sawyerId ,[0,1,2,3,4,5,6,7,8]))) -- 9
    jointStates = p.getJointStates(sawyerId, [1,3,4,5,6,7,8])
    if jointStates is None:
        continue
    raw_q = [jointState[0] for jointState in jointStates]
    raw_dq = [jointState[1] for jointState in jointStates]
    q = "{} {} {} {} {} {} {}".format(raw_q[0], raw_q[1], raw_q[2], raw_q[3], raw_q[4], raw_q[5], raw_q[6])
    dq = "{} {} {} {} {} {} {}".format(raw_dq[0], raw_dq[1], raw_dq[2], raw_dq[3], raw_dq[4], raw_dq[5], raw_dq[6])
    print('q:::',q)


    # Wait for Sawyer Controller to finish reading previous Q Value
    q_isReady = r.get(PY_Q_READY)


    while q_isReady is not None and q_isReady == "Ready to Read":
        q_isReady = r.get(PY_Q_READY)

    #Write values to Redis Client
    r.set(PY_JOINT_ANGLES_KEY, q)
    r.set(PY_JOINT_VELOCITIES_KEY, dq)
    #r.set(PY_EE_DES_POSITION_KEY, pos)
    r.set(PY_Q_READY, "Ready to Read")

    #Wait for torque value to be ready
    torque_isReady = r.get(PY_TORQUE_READY)
    while torque_isReady != "Ready to Read":
        torque_isReady = r.get(PY_TORQUE_READY)

    #Read torque values
    tau = r.get(PY_JOINT_TORQUES_COMMANDED_KEY) # Get torque from PID

    #Tell Saw Controller that torque value has been read
    r.set(PY_TORQUE_READY, "Ready to Write")

    #Build float array
    tau_floats = [float(x) for x in tau.split()]
    print(tau_floats)


    #Apply torques
    p.setJointMotorControlArray(sawyerId, [1,3,4,5,6,7,8],
                            controlMode=p.TORQUE_CONTROL,
                            physicsClientId=physicsClient,
                            forces=tau_floats)

    p.stepSimulation()


p.disconnect()

