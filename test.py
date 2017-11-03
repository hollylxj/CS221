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
#urdfFile = "/Users/holly/sawyer.git/bin/resources/sawyer/sawyer.urdf"
urdfFile = "/Users/holly/bullet3/data/sawyer_robot/sawyer_description/urdf/sawyer.urdf"
#print(os.getcwd())
#os.chdir("/Users/holly/sawyer.git/bin/resources/sawyer")
#urdfFile = "sawyer_robot/sawyer_description/urdf/sawyer.urdf"

sawyer.setup(gravity, timeStep, urdfFile)
#sawyerId = p.loadURDF("sawyer.urdf", useFixedBase = 1)
#p.setRealTimeSimulation(1,sawyerId)
raw_input("Press Enter to continue...")


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
while(1):
    sawyer.moveTo((0,-0.5,0.5,0.5,0.5,0.5,0))
    time.sleep(1)
    sawyer.moveTo((0,-0.5,-0.5,-0.5,-0.5,0.5,0))
    time.sleep(1)

