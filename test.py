import os
import sawyer
from math import pi
import time

##########################
## Setup Sawyer simulator
##########################
#physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
sawyer.connect()
gravity = (0, 0, -9.8)
timeStep = 0.0001
#urdfFile = "sawyer.urdf"
# urdfFile = "/Users/holly/CS221/sawyer_no_base.urdf"
# urdfFile = "/Users/holly/bullet3/data/sawyer_robot/sawyer_description/urdf/sawyer.urdf"
urdfFile = "/Users/holly/CS221/rethink/sawyer_description/urdf/sawyer_with_gripper.urdf"
#print(os.getcwd())
#os.chdir("/Users/holly/sawyer.git/bin/resources/sawyer")
#urdfFile = "sawyer_robot/sawyer_description/urdf/sawyer.urdf"

sawyer.setup(gravity, timeStep, urdfFile)
#sawyerId = p.loadURDF("sawyer.urdf", useFixedBase = 1)
#p.setRealTimeSimulation(1,sawyerId)
# input("Press Enter to continue...")


############################
## Reseting initial position
############################
sawyer.resetPos(0.5)
#time.sleep(1)


#######################
## input desired position and orientation
#######################
pos = (0.8, 0.0, 0.0)
orn = (1, 0, 0)


###################
### TODO: ./sawyer sawyer.urdf
### Run sawyer.cpp
####################


########################
## START MOVING
########################
i=0
while(1):
    xyz_pos =  sawyer.getTargetPosition()
    print(xyz_pos)
    pos = sawyer.getTargetJointPosition(xyz_pos)
    sawyer.moveTo(pos)
    #input("Press Enter to continue...")
#time.sleep(1)
#sawyer.moveTo((0.5,-0.5,0,0,-0.5,0.5,0))
#time.sleep(1)



