import os
import sawyer
from math import pi
import time
import random
from operator import add
import numpy as np
from rla import *
from itertools import combinations



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
urdfFile = "rethink/sawyer_description/urdf/sawyer_with_gripper.urdf"
#print(os.getcwd())
#os.chdir("/Users/holly/sawyer.git/bin/resources/sawyer")
#urdfFile = "sawyer_robot/sawyer_description/urdf/sawyer.urdf"

jointLimits = sawyer.setup(gravity, timeStep, urdfFile)
#sawyerId = p.loadURDF("sawyer.urdf", useFixedBase = 1)
#p.setRealTimeSimulation(1,sawyerId)
# input("Press Enter to continue...")


############################
## Reseting initial position
############################
#sawyer.resetPos(0.5)
#time.sleep(1)


#######################
## input desired position and orientation
#######################
pos = (0.8, 0.0, 0.0)
orn = (1, 0, 0)
action_space = [[0.1,0,0,0,0,0,0],[0,0.1,0,0,0,0,0],[0,0,0.1,0,0,0,0],[0,0,0,0.1,0,0,0],[0,0,0,0,0.1,0,0],[0,0,0,0,0,0.1,0],[0,0,0,0,0,0,0.1],
                    [-0.1,0,0,0,0,0,0],[0,-0.1,0,0,0,0,0],[0,0,-0.1,0,0,0,0],[0,0,0,-0.1,0,0,0],[0,0,0,0,-0.1,0,0],[0,0,0,0,0,-0.1,0],[0,0,0,0,0,0,0.1]
                    ]


###################
### TODO: ./sawyer sawyer.urdf
### Run sawyer.cpp
####################


########################
## START MOVING
########################
i=0

xyz_pos =  sawyer.getTargetPosition()
initial_state = sawyer.readQ()
s = sawyer.readQ()
#print(xyz_pos)
print(initial_state)

def myQLearning(qla, numTrials=10, maxIterations=1000, verbose=False, test = False):
    DOF = len(initial_state)
    
    totalRewards = []  # The rewards we get on each trial
    for trial in range(numTrials):

        #sawyer initialization
        state = tuple(initial_state)
        sawyer.resetPos(initial_state)
        sawyer.resetCube()
        #raw_input("DEBUG: begin trial")

        #logging initializations
        sequence = [state]
        totalDiscount = 1
        totalReward = 0
        for count in range(maxIterations):
            #print("DEBUG: State=",state)
            #get action based on exploration policy
            # ACTION FOR Q LEARNING
            action = qla.getAction(state)

            #print("DEBUG: Action=",action)
            successor = tuple(state[i]+action[i] for i in range(DOF))
            #print("DEBUG: Succ=",successor)
            terminalState = False

            #Check for dead angle
            for i in range(DOF):
                if(successor[i]>jointLimits[i][1] or successor[i]<jointLimits[i][0]):
                    reward = reward-100
                    terminalState = True
                    break

            if not terminalState:
                if not test:
                    if sawyer.moveTo(successor):
                        terminalState, reward = sawyer.checkCollision()
                    else:
                        terminalState, reward = sawyer.checkCollision()
                        #reward = reward-100
                        
                        if reward > -100:
                            reward = reward -100
                        terminalState = True
                else:
                    if sawyer.moveTo_test(successor):
                        terminalState, reward = sawyer.checkCollision()
                    else:
                        terminalState, reward = sawyer.checkCollision()
                        reward = reward -100
                        if reward > -100:
                            reward = reward -100
                        terminalState = True
            
            if reward > 0 or reward <=-100:
                print("reward for trial ", trial," out of ", numTrials," iteration ", count,"= ",reward)

            totalReward += reward

            if terminalState:
                qla.incorporateFeedback(state, action, reward, None)
                break;
            qla.incorporateFeedback(state, action, reward, successor)

            state = successor

        if verbose:
            print("Trial %d (totalReward = %s): %s" % (trial, totalReward, sequence))
        totalRewards.append(totalReward)

        
    return totalRewards

# Should return a list of (feature key, feature value) pairs.
# Include the following features in the list you return:
# --
# --
#       Note:
def sawyerFeatureExtractor(state, action):
    
    features = []
    DOF = len(state)
    MAX_DEGREE = 1

    #Polynomial joint features
    for n in range(1, MAX_DEGREE+1):
        for i in range(DOF):
            features.append( (str(tuple([i]*n)), (state[i]+pi)**n) )

    #Compound-joint features
    #for i in range(2, DOF+1):
    for i in range(2, 3):
        for joints in combinations(range(DOF), i):
            value = 1
            for joint in joints:
                value *= (state[joint]+pi)
            features.append( (str(joints), value) )

    #print("DEBUG: Features=", features)
    #raw_input()
    return features
# END_YOUR_CODE



def main():
    MAX_ITER = 200
    LEARNING_TRIALS = 1000
    TESTING_TRIALS = 1
    


    #Learn with epsilon = 0.5
    qla = QLearningAlgorithm(actions = sawyer.getActions, discount = 1, explorationProb = 0.8)
    #qla = QLearningApproxAlgorithm(actions = sawyer.getActions, discount = 1, featureExtractor=sawyerFeatureExtractor, explorationProb = 0.5)
    #qla.load()

    totalRewards = myQLearning(qla, numTrials=LEARNING_TRIALS, maxIterations=MAX_ITER, test=False)
    ##print("Total Rewards:", totalRewards)

    input("Finish Learning")

    ##Act optimally by setting epsilon = 0
    qla.explorationProb = 0
    totalRewards = myQLearning(qla, numTrials=TESTING_TRIALS, maxIterations=MAX_ITER,test=True)
    print("Total Rewards:", totalRewards)
    #print("Average Rewards:", sum(totalRewards)/TESTING_TRIALS)
    input("Finish Testing")
    #print("Q-Learning Completed")
    #print("Number of States explored:", len(qla.Q))



main()



