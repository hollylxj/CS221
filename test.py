import pybullet as p
import os
import time
import sawyer
from math import pi




##########################
## Setup Sawyer simulator
##########################
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setGravity(0,0,-9.8)
p.setTimeStep(0.0002)
#print(os.getcwd())
os.chdir("/Users/holly/sawyer.git/bin/resources/sawyer")
sawyerId = p.loadURDF("sawyer.urdf", useFixedBase = 1)
#p.setRealTimeSimulation(1,sawyerId)



############################
## Reseting initial position
############################
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



#######################
#Preparation for torque mode control
#######################
p.setJointMotorControlArray(sawyerId, jointIndices=jids,
                            controlMode=p.VELOCITY_CONTROL,
                            physicsClientId=physicsClient,
                            forces=(0.,) * len(jids) )
# set 0 Torque, initial Q and dQ
#r.set(PY_JOINT_TORQUES_COMMANDED_KEY,"{} {} {} {} {} {} {}".format(0,0,0,0,0,0,0))
#r.set(PY_JOINT_ANGLES_KEY, q)
#r.set(PY_JOINT_VELOCITIES_KEY, dq)


#######################
## input desired position and orientation
#######################
pos = "{} {} {}".format(0.0, 0.8, 0.0)
orn = "{} {} {}".format(0.1, 0.1, 0.1)
sawyer.setDesPosOrn(pos,orn)



### Run sawyer.cpp

########################
## START MOVING
########################
sawyer.moveTo(physicsClient,sawyerId)



p.disconnect()

