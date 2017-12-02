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
urdfFile = "/Users/holly/CS221/rethink/sawyer_description/urdf/sawyer_with_gripper.urdf"
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
input("Press Enter to continue...")
        #break
#input("Press Enter to continue...")
#time.sleep(1)
#sawyer.moveTo((0.5,-0.5,0,0,-0.5,0.5,0))
#time.sleep(1)

def myQLearning(qla, numTrials=10, maxIterations=1000, verbose=False):
    DOF = len(initial_state)
    
    totalRewards = []  # The rewards we get on each trial
    for trial in range(numTrials):

        #sawyer initialization
        state = tuple(initial_state)
        sawyer.moveTo(initial_state)
        sawyer.resetCube()
        #input("DEBUG: begin trial")

        #logging initializations
        sequence = [state]
        totalDiscount = 1
        totalReward = 0
        for _ in range(maxIterations):
            #print("DEBUG: State=",state)
            #get action based on exploration policy
            action = qla.getAction(state)
            #print("DEBUG: Action=",action)
            successor = tuple(state[i]+action[i] for i in range(DOF))
            #print("DEBUG: Succ=",successor)
            terminalState = False

            #Check for dead angle
            for i in range(DOF):
                if(successor[i]>jointLimits[i][1] or successor[i]<jointLimits[i][0]):
                    reward = -50
                    terminalState = True
                    break

            if not terminalState:
                if sawyer.moveTo(successor):
                    terminalState, reward = sawyer.checkCollision()
                else:
                    reward = -100
                    terminalState = True
                    
            #logging
            sequence.append(action)
            sequence.append(reward)
            sequence.append(successor)
            totalReward += totalDiscount * reward
            totalDiscount *= qla.discount
            #print("DEBUG: Reward=",reward)
            #incorporate (s,a,r,s')
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
    #input()
    return features
# END_YOUR_CODE

def Qlearning():
    gamma = 1
    alpha = 0.9
    Q = dict()
    #N = dict()
    R = dict()
    action_space = [[0.1,0,0,0,0,0,0],[0,0.1,0,0,0,0,0],[0,0,0.1,0,0,0,0],[0,0,0,0.1,0,0,0],[0,0,0,0,0.1,0,0],[0,0,0,0,0,0.1,0],[0,0,0,0,0,0,0.1],
                    [-0.1,0,0,0,0,0,0],[0,-0.1,0,0,0,0,0],[0,0,-0.1,0,0,0,0],[0,0,0,-0.1,0,0,0],[0,0,0,0,-0.1,0,0],[0,0,0,0,0,-0.1,0],[0,0,0,0,0,0,0.1]
                    ]
    # s0 and a0
    s = sawyer.readQ()
    a = action_space[random.choice(range(len(action_space)))]
    
    while(1):
        #pos = sawyer.getTargetJointPosition(xyz_pos)
        #sawyer.moveTo([0.1,0,0,0,0,0,0])
        sawyer.moveTo(s)
        print("Debug:::::1")
        s_cur = repr(s)
        if s_cur not in R.keys():
            R[s_cur] = sawyer.checkCollision()
        #print(s)



        #successor states
        successors = [[s[i]+action[i] for i in range(len(a))] for action in action_space]
        #input("Press Enter to continue...")
        
        # explore strategy: soft max
        
        #explore = list(range(len(action_space)))
        #explore.remove(7) # do not come back
        #rand = random.choice(explore) % 14
        #a_next = a[rand:]+a[:rand]

        #Exploration using epsilon-greedy
        rand = random.randint(0, 100)
        epsilon = 52
        if rand >= epsilon:
            M = [0 for x in range(len(successors))]
            for i in range(len(successors)):
               x = repr(successors[i])
               if x not in Q.keys():
                   M[i] = -1000000
               else:
                   M[i] = Q[x]
            s_next = successors[np.argmax(M)]
            a = action_space[np.argmax(M)]
        else:
            a = action_space[rand%13]
            s_next = [s[i]+a[i] for i in range(len(a))]

        s_a = repr(s+a)
        s_next_a = repr(s_next+a)

        # check if next state is within joint limits
        for i in range(len(s_next)):
            #print("i ", i)
            if(s_next[i]>jointLimits[i][1] or s_next[i]<jointLimits[i][0]):
                R[s_cur] = -5




        if s_a not in Q.keys():
            Q[s_a] = 0
            #N[s_a] = 1
            #delta = r
            if s_next_a in Q.keys():
                Q[s_a] = alpha*(R[s_cur]+gamma*Q[s_next_a])
            else:
                Q[s_a] = alpha*R[s_cur]
    
        else:
            #N[s_a] += 1
            #delta = r + gamma * Q[s_a_next] - Q[s_a]
            if s_next_a in Q.keys():
                Q[s_a] += alpha*(R[s_cur]+gamma*Q[s_next_a]-Q[s_a])
            else:
                Q[s_a] += alpha*(R[s_cur]-Q[s_a])
        #print(Q[s_a])
        #Q[s_a] += alpha * gamma * N[s_a]
        print("current Q= ",Q[s_a])
        if R[s_cur] == 1000:
            #sawyer.resetPos(0.5)
            s = initial_state
            sawyer.moveTo(s)
            a = action_space[random.choice(range(len(action_space)))]
            sawyer.resetCube()
            print("success!")
            #return Q
        elif R[s_cur] == -10:
            #sawyer.resetPos(0.5)
            s = initial_state
            sawyer.moveTo(s)
            a = action_space[random.choice(range(len(action_space)))]
            #return Q
            print("collision!")
        elif R[s_cur] == -5:
            s = initial_state
            sawyer.moveTo(s)
            a = action_space[random.choice(range(len(action_space)))]
            print("dead angle!")
        else:
            #a = a_next
            s = s_next

        #return Q
        #print(s)


def main():
    MAX_ITER = 100000
    LEARNING_TRIALS = 100000
    TESTING_TRIALS = 3
    
    #Learn with epsilon = 0.5
    qla = QLearningAlgorithm(actions = sawyer.getActions, discount = 1, explorationProb = 0.5)
    #qla = QLearningApproxAlgorithm(actions = sawyer.getActions, discount = 1, featureExtractor=sawyerFeatureExtractor, explorationProb = 0.5)

    #print("Before Load:", qla.Q)
    #print("Before Load:", qla.weights)
    qla.load()
    #print("After Load:", qla.Q)
    #print("After Load:", qla.weights)
    #print("Size of Weight vector:",len(qla.weights))
    #input()

    totalRewards = myQLearning(qla, numTrials=LEARNING_TRIALS, maxIterations=MAX_ITER)
    print("Total Rewards:", totalRewards)
    input("Finish Learning")

    #print("Weights after training:", qla.Q)
    #print("Size of Weight vector:", len(qla.Q))

    #print("Weights after training:", qla.weights)
    #print("Size of Weight vector:", len(qla.weights))
    qla.save()
    
    #Act optimally by setting epsilon = 0
    qla.explorationProb = 0
    totalRewards = myQLearning(qla, numTrials=TESTING_TRIALS, maxIterations=MAX_ITER)
    print("Total Rewards:", totalRewards)
    print("Average Rewards:", sum(totalRewards)/TESTING_TRIALS)
    input("Finish Testing")
    print("Q-Learning Completed")
    #print("Number of States explored:", len(qla.Q))

def Demo():
    xyz_pos =  sawyer.getTargetPosition()
#print(xyz_pos)
    while(1):
        pos = sawyer.getTargetJointPosition(xyz_pos)
        sawyer.moveTo(pos)
        sawyer.checkCollision()
    #if sawyer.checkCollision() == 1000:
        #print("collision!")
        #break

main()
#Qlearning()
#Demo()



