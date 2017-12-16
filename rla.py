import collections
import random
import math
import json

############################################################

# An algorithm that solves an MDP (i.e., computes the optimal
# policy).
class MDPAlgorithm:
    # Set:
    # - self.pi: optimal policy (mapping from state to action)
    # - self.V: values (mapping from state to best values)
    def solve(self, mdp): raise NotImplementedError("Override me")

############################################################
class ValueIteration(MDPAlgorithm):
    '''
    Solve the MDP using value iteration.  Your solve() method must set
    - self.V to the dictionary mapping states to optimal values
    - self.pi to the dictionary mapping states to an optimal action
    Note: epsilon is the error tolerance: you should stop value iteration when
    all of the values change by less than epsilon.
    The ValueIteration class is a subclass of util.MDPAlgorithm (see util.py).
    '''
    def solve(self, mdp, epsilon=0.001):
        mdp.computeStates()
        def computeQ(mdp, V, state, action):
            # Return Q(state, action) based on V(state).
            return sum(prob * (reward + mdp.discount() * V[newState]) \
                            for newState, prob, reward in mdp.succAndProbReward(state, action))

        def computeOptimalPolicy(mdp, V):
            # Return the optimal policy given the values V.
            pi = {}
            for state in mdp.states:
                pi[state] = max((computeQ(mdp, V, state, action), action) for action in mdp.actions(state))[1]
            return pi

        V = collections.defaultdict(float)  # state -> value of state
        numIters = 0
        while True:
            newV = {}
            for state in mdp.states:
                # This evaluates to zero for end states, which have no available actions (by definition)
                newV[state] = max(computeQ(mdp, V, state, action) for action in mdp.actions(state))
            numIters += 1
            if max(abs(V[state] - newV[state]) for state in mdp.states) < epsilon:
                V = newV
                break
            V = newV

        # Compute the optimal policy now
        pi = computeOptimalPolicy(mdp, V)
        print( "ValueIteration: %d iterations" % numIters )
        self.pi = pi
        self.V = V

# An abstract class representing a Markov Decision Process (MDP).
class MDP:
    # Return the start state.
    def startState(self): raise NotImplementedError("Override me")

    # Return set of actions possible from |state|.
    def actions(self, state): raise NotImplementedError("Override me")

    # Return a list of (newState, prob, reward) tuples corresponding to edges
    # coming out of |state|.
    # Mapping to notation from class:
    #   state = s, action = a, newState = s', prob = T(s, a, s'), reward = Reward(s, a, s')
    # If IsEnd(state), return the empty list.
    def succAndProbReward(self, state, action): raise NotImplementedError("Override me")

    def discount(self): raise NotImplementedError("Override me")

    # Compute set of states reachable from startState.  Helper function for
    # MDPAlgorithms to know which states to compute values and policies for.
    # This function sets |self.states| to be the set of all states.
    def computeStates(self):
        self.states = set()
        queue = []
        self.states.add(self.startState())
        queue.append(self.startState())
        while len(queue) > 0:
            state = queue.pop()
            for action in self.actions(state):
                for newState, prob, reward in self.succAndProbReward(state, action):
                    if newState not in self.states:
                        self.states.add(newState)
                        queue.append(newState)
        # print( "%d states" % len(self.states) )
        # print( self.states )

############################################################

# A simple example of an MDP where states are integers in [-n, +n].
# and actions involve moving left and right by one position.
# We get rewarded for going to the right.
class NumberLineMDP(MDP):
    def __init__(self, n=5): self.n = n
    def startState(self): return 0
    def actions(self, state): return [-1, +1]
    def succAndProbReward(self, state, action):
        return [(state, 0.4, 0),
                (min(max(state + action, -self.n), +self.n), 0.6, state)]
    def discount(self): return 0.9

############################################################




############################################################

# Abstract class: an RLAlgorithm performs reinforcement learning.  All it needs
# to know is the set of available actions to take.  The simulator (see
# simulate()) will call getAction() to get an action, perform the action, and
# then provide feedback (via incorporateFeedback()) to the RL algorithm, so it can adjust
# its parameters.
class RLAlgorithm:
    # Your algorithm will be asked to produce an action given a state.
    def getAction(self, state): raise NotImplementedError("Override me")

    # We will call this function when simulating an MDP, and you should update
    # parameters.
    # If |state| is a terminal state, this function will be called with (s, a,
    # 0, None). When this function is called, it indicates that taking action
    # |action| in state |state| resulted in reward |reward| and a transition to state
    # |newState|.
    def incorporateFeedback(self, state, action, reward, newState): raise NotImplementedError("Override me")

# An RL algorithm that acts according to a fixed policy |pi| and doesn't
# actually do any learning.
class FixedRLAlgorithm(RLAlgorithm):
    def __init__(self, pi): self.pi = pi

    # Just return the action given by the policy.
    def getAction(self, state): return self.pi[state]

    # Don't do anything: just stare off into space.
    def incorporateFeedback(self, state, action, reward, newState): pass

# Performs Q-learning.  Read rla.RLAlgorithm for more information.
# actions: a function that takes a state and returns a list of actions.
# discount: a number between 0 and 1, which determines the discount factor
# featureExtractor: a function that takes a state and action and returns a list of (feature name, feature value) pairs.
# explorationProb: the epsilon value indicating how frequently the policy
# returns a random action
class QLearningAlgorithm(RLAlgorithm):
    def __init__(self, actions, discount, explorationProb=0.2):
        self.actions = actions
        self.discount = discount
        self.explorationProb = explorationProb
        self.numIters = 0
        self.Q = {}
        random.seed(0)
    
    def load(self, file=None):
        if file is None:
            file = "qvalues.txt"
        self.Q = json.load(open(file))

    def save(self, file=None):
        if file is None:
            file = "qvalues.txt"
        json.dump(self.Q, open(file,'w'))
    
    # Return the Q function associated with the weights and features
    def getQ(self, state, action):
        
        state = str(state)
        action = str(action)

        if state not in self.Q:
            self.Q[state] = {}
        if action not in self.Q[state]:
            self.Q[state][action] = 0.0
        return self.Q[state][action]
    
    # This algorithm will produce an action given a state.
    # Here we use the epsilon-greedy algorithm: with probability
    # |explorationProb|, take a random action.
    def getAction(self, state):
        self.numIters += 1
        
        if random.random() < self.explorationProb:
            return random.choice(self.actions(state))
        else:
            return max((self.getQ(state, action), action) for action in self.actions(state))[1]

    # Call this function to get the step size to update the weights.
    def getStepSize(self):
        #return 1.0 / math.sqrt(self.numIters)
        return 0.5
        
    # We will call this function with (s, a, r, s'), which you should use to update |weights|.
    # Note that if s is a terminal state, then s' will be None.  Remember to check for this.
    # You should update the weights using self.getStepSize(); use
    # self.getQ() to compute the current estimate of the parameters.
    def incorporateFeedback(self, state, action, reward, newState):
        
        prediction = self.getQ(state, action)
        
        #if state is terminal state, newState would be None
        if newState is None:
            Vopt = 0
        else:
            Vopt = max( self.getQ(newState, a) for a in self.actions(newState) )
        
        target = reward + self.discount * Vopt
        eta = self.getStepSize()
        self.Q[str(state)][str(action)] = (1-eta)*prediction + eta*target

        #if reward > 1:
        #   print("reward = ", reward)
        #   print("Vopt = ", Vopt)
        #   print("Target = ", target)
        #   print("prediction = ", prediction)
        #   print("new Q = ", self.Q[str(state)][str(action)])

# Performs Q-learning with Approximation.  Read rla.RLAlgorithm for more information.
# actions: a function that takes a state and returns a list of actions.
# discount: a number between 0 and 1, which determines the discount factor
# featureExtractor: a function that takes a state and action and returns a list of (feature name, feature value) pairs.
# explorationProb: the epsilon value indicating how frequently the policy
# returns a random action
class QLearningApproxAlgorithm(RLAlgorithm):
    def __init__(self, actions, discount, featureExtractor, explorationProb=0.2):
        self.actions = actions
        self.discount = discount
        self.featureExtractor = featureExtractor
        self.explorationProb = explorationProb
        self.weights = collections.defaultdict(float)
        self.numIters = 0
        random.seed(0)
    
    def load(self, file=None):
        if file is None:
            file = "weights.txt"
        self.weights = json.load(open(file))
    
    def save(self, file=None):
        if file is None:
            file = "weights.txt"
        json.dump(self.weights, open(file,'w'))
    
    # Return the Q function associated with the weights and features
    def getQ(self, state, action):
        score = 0
        for f, v in self.featureExtractor(state, action):
            score += self.weights[f] * v
        return score
    
    # This algorithm will produce an action given a state.
    # Here we use the epsilon-greedy algorithm: with probability
    # |explorationProb|, take a random action.
    def getAction(self, state):
        self.numIters += 1
        if random.random() < self.explorationProb:
            return random.choice(self.actions(state))
        else:
            return max((self.getQ(state, action), action) for action in self.actions(state))[1]

    # Call this function to get the step size to update the weights.
    def getStepSize(self):
        #return 1.0 / math.sqrt(self.numIters)
        return 0.2
    
    # We will call this function with (s, a, r, s'), which you should use to update |weights|.
    # Note that if s is a terminal state, then s' will be None.  Remember to check for this.
    # You should update the weights using self.getStepSize(); use
    # self.getQ() to compute the current estimate of the parameters.
    def incorporateFeedback(self, state, action, reward, newState):
        
        prediction = self.getQ(state, action)
        
        #if state is terminal state, newState would be None
        if newState is None:
            Vopt = 0
        else:
            Vopt = max( self.getQ(newState, a) for a in self.actions(newState) )
        
        target = reward + self.discount * Vopt
        difference = prediction - target
        eta = self.getStepSize()
        eta_diff = eta * difference
        if difference > 0:
            print("s,a,r,s'=",state, action, reward, newState)
            print("Prediction=",prediction)
            print("target=",target)
            print("difference=",difference)
            print("weights=", self.weights)
            #input("Hit to update weights")
        for f, v in self.featureExtractor(state, action):
            if difference > 0:
                print("eta*diff*v=",eta_diff * v)
            self.weights[f] -= eta_diff * v
        if difference > 0:
            print("weights=", self.weights)
            #input("Weights updated")

# Performs Q-Lambda learning.  Read rla.RLAlgorithm for more information.
# actions: a function that takes a state and returns a list of actions.
# discount: a number between 0 and 1, which determines the discount factor
# featureExtractor: a function that takes a state and action and returns a list of (feature name, feature value) pairs.
# explorationProb: the epsilon value indicating how frequently the policy
# returns a random action
class QLambdaLearningAlgorithm(RLAlgorithm):
    def __init__(self, actions, discount, decay, explorationProb=0.2):
        self.actions = actions
        self.discount = discount
        self.explorationProb = explorationProb
        self.numIters = 0
        self.Q = {}
        self.N = {}
        self.decay = decay
        random.seed(0)
    
    def load(self, file=None):
        if file is None:
            file = "qlambdavalues.txt"
        self.Q = json.load(open(file))
    
    def save(self, file=None):
        if file is None:
            file = "qlambdavalues.txt"
        json.dump(self.Q, open(file,'w'))
    
    # Return the Q function associated with the weights and features
    def getQ(self, state, action):
        
        state = str(state)
        action = str(action)
        
        if state not in self.Q:
            self.Q[state] = {}
        if action not in self.Q[state]:
            self.Q[state][action] = 0.0
        return self.Q[state][action]
    
    # This algorithm will produce an action given a state.
    # Here we use the epsilon-greedy algorithm: with probability
    # |explorationProb|, take a random action.
    def getAction(self, state):
        self.numIters += 1
        if random.random() < self.explorationProb:
            return random.choice(self.actions(state))
        else:
            return max((self.getQ(state, action), action) for action in self.actions(state))[1]

    # Call this function to get the step size to update the weights.
    def getStepSize(self):
        #return 1.0 / math.sqrt(self.numIters)
        return 0.5
    
    # We will call this function with (s, a, r, s'), which you should use to update |weights|.
    # Note that if s is a terminal state, then s' will be None.  Remember to check for this.
    # You should update the weights using self.getStepSize(); use
    # self.getQ() to compute the current estimate of the parameters.
    def incorporateFeedback(self, state, action, reward, newState):
        
        prediction = self.getQ(state, action)
        if state not in self.N:
            self.N[state] = {}
        if action not in self.N[state]:
            self.N[state][action] = 1.0
        else:
            self.N[state][action] += 1
        
        #if state is terminal state, newState would be None
        if newState is None:
            Vopt = 0
        else:
            Vopt = max( self.getQ(newState, a) for a in self.actions(newState) )
        
        target = reward + self.discount * Vopt
        diff = target - prediction
        eta = self.getStepSize()
        #if reward > 0:
        #if True:
        for nstate in self.N:
            for naction in self.N[nstate]:
                self.Q[str(nstate)][str(naction)] += eta*diff*self.N[nstate][naction]
                self.N[nstate][naction] *= self.discount*self.decay


        if newState is None:
            self.N = {}

############################################################

# Perform |numTrials| of the following:
# On each trial, take the MDP |mdp| and an RLAlgorithm |rl| and simulates the
# RL algorithm according to the dynamics of the MDP.
# Each trial will run for at most |maxIterations|.
# Return the list of rewards that we get for each trial.
def simulate(mdp, rl, numTrials=10, maxIterations=1000, verbose=False,
             sort=False):
    random.seed(2)
    # Return i in [0, ..., len(probs)-1] with probability probs[i].
    def sample(probs):
        target = random.random()
        accum = 0
        for i, prob in enumerate(probs):
            accum += prob
            if accum >= target: return i
        raise Exception("Invalid probs: %s" % probs)

    totalRewards = []  # The rewards we get on each trial
    for trial in range(numTrials):
        state = mdp.startState()
        sequence = [state]
        totalDiscount = 1
        totalReward = 0
        for _ in range(maxIterations):
            action = rl.getAction(state)
            transitions = mdp.succAndProbReward(state, action)
            if sort: transitions = sorted(transitions)
            if len(transitions) == 0:
                rl.incorporateFeedback(state, action, 0, None)
                break

            # Choose a random transition
            i = sample([prob for newState, prob, reward in transitions])
            newState, prob, reward = transitions[i]
            sequence.append(action)
            sequence.append(reward)
            sequence.append(newState)

            rl.incorporateFeedback(state, action, reward, newState)
            totalReward += totalDiscount * reward
            totalDiscount *= mdp.discount()
            state = newState
        if verbose:
            print( "Trial %d (totalReward = %s): %s" % (trial, totalReward, sequence) )
        totalRewards.append(totalReward)
    return totalRewards
