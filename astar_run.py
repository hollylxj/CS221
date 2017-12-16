import astar as a
import pybullet as p
import pybullet_data

def distance(p1, p2):
    result = 0
    for i in range(len(p1)):
        result += (p1[i] - p2[i]) ** 2
    return result ** 0.5

class pathFindingProblem:
    def __init__(self, sawyerId, target, jointLimits):
        #Simple version. Consider using a layouts file in the future
        self.startPos =  p.getLinkState(sawyerId,7)[0]
        self.sawyerId = sawyerId
        self.target = tuple(target)
        self.jointLimits = jointLimits
        self.axes = ('x', 'y', 'z')

    def startState(self):
        return tuple(self.startPos)

    def isEnd(self, state):
        return distance(state, self.target) < 0.1

    def succAndCost(self, state):
        #global ik_time
        #global move_time
        #global eval_time
        options = []
        for i in range(3): #Choose an axis
            for stepSize in [0.1, -0.1]:
                newState = list(state)
                newState[i] += stepSize
                if i == 2 and newState[i] < 0.1:
                    continue
                newPos = p.calculateInverseKinematics(self.sawyerId,7,newState, \
                    lowerLimits=self.jointLimits['lower'], upperLimits=self.jointLimits['upper'])
                for _ in range(len(newPos)):
                    p.resetJointState(self.sawyerId, _, newPos[_])
                #newState = p.getLinkState(self.sawyerId,7)[0]
                p.stepSimulation()
                if not p.getContactPoints(self.sawyerId):
                    futureCost = distance(newState, self.target)
                    cost = 1 + futureCost - distance(state, self.target)
                    options.append((self.axes[i]+'+', tuple(newState), cost, futureCost))
        return options

#initialArmPos = [1.135500106476994, 0.16303487890749857, 0.3325096500679113]
#targetPos = [0,1,0.1]
targetPos = [-0.7,0.7,0.1]
#obstaclePos = [0.53550010647699413, 0.5630348789074985, 0.23250965006791127]
obstaclePos = [1.1, 1.1, 0.9]

def setup(target, obstacle=None):
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,0)
    p.setTimeStep(1)
    planeId = p.loadURDF("plane.urdf")
    sawyerId = p.loadURDF("/rethink/sawyer_description/urdf/sawyer_with_gripper.urdf", useFixedBase=1,flags=2)
    targetId = p.loadURDF('sphere2.urdf', targetPos, globalScaling = 0.1)
    if obstacle:
        obstacleId = p.loadURDF('cube.urdf', obstacle, useFixedBase=1, flags=2)
    jointLimits = {'lower':[],'upper':[]}
    for i in range(14):
        info = p.getJointInfo(sawyerId,i)
        jointLimits['lower'].append(info[8])
        jointLimits['upper'].append(info[9])
    return sawyerId, jointLimits

##########################
## Path finding
##########################
p.connect(p.DIRECT)
sawyerId, jointLimits = setup(targetPos, obstaclePos)

astar = a.AStarSearch(1)
problem = pathFindingProblem(sawyerId, targetPos, jointLimits)

#ik_time = 0
#move_time = 0
#eval_time = 0
pathStates, exploredStates, frontierStates = astar.solve(problem)
#print ik_time
#print move_time
#print eval_time
print("statesOnPath = {}".format(len(pathStates)))
#input("Press Enter to continue...")

p.disconnect()

#######################
## input desired position and orientation
#######################
#pos = [[1,0,0],[2,0,0],[3,0,0],[3,0,3],[3,3,3]]
#orn = (1, 0, 0)

########################
## Actual Moving
########################
p.connect(p.GUI)
sawyerId, jointLimits = setup(targetPos, obstaclePos)

#p.addUserDebugLine(initialArmPos, pathStates[0], [0,0,255])
#for i in range(len(pathStates) - 1):
#    p.addUserDebugLine(pathStates[i], pathStates[i+1], [0,0,255])
#input("Press Enter to continue...")

pathStates = list(pathStates)
newPos = p.getLinkState(sawyerId,7)[0]
while pathStates:
    oldPos = newPos
    nextState = list(pathStates.pop(0))
    nextJoints = p.calculateInverseKinematics(sawyerId,7,nextState, \
        lowerLimits=jointLimits['lower'], upperLimits=jointLimits['upper'])
    for _ in range(len(nextJoints)):
        p.resetJointState(sawyerId, _, nextJoints[_])
    newPos = p.getLinkState(sawyerId,7)[0]
    p.addUserDebugLine(oldPos, newPos, [0,0,255])
    #input("Press Enter to continue...")
    #command = raw_input("Press Enter to continue...")
    #if command == 'y':
    #    print nextState
input("Press Enter to continue...")
p.disconnect()
