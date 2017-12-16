import heapq

class AStarSearch:
    def __init__(self, verbose=0):
        self.verbose = verbose

    def solve(self, problem):
        # If a path exists, set |actions| and |totalCost| accordingly.
        # Otherwise, leave them as None.
        self.actions = None
        self.totalCost = None
        self.numStatesExplored = 0
        self.exploredStates = []
        self.frontierStates = []

        # Initialize data structures
        frontier = PriorityQueue()  # Explored states are maintained by the frontier.
        backpointers = {}  # map state to (action, previous state)

        # Add the start state
        startState = problem.startState()
        frontier.update(startState, 0, float('inf'))

        while True:
            #start_time = time.time()
            # Remove the state from the queue with the lowest pastCost
            # (priority).
            state, pastCost, priority = frontier.removeMin()
            if len(backpointers) > 1:
                self.exploredStates.append([backpointers[state][1], state])

            if state == None: break
            self.numStatesExplored += 1
            #if self.verbose >= 2:
                #print "Exploring %s with pastCost %s and heuristic %s" % (state, pastCost, priority - pastCost)

            # Check if we've reached an end state; if so, extract solution.
            if problem.isEnd(state):
                self.actions = []
                self.states = []
                while state != startState:
                    action, prevState = backpointers[state]
                    self.actions.append(action)
                    self.states.append(state)
                    state = prevState
                self.actions.reverse()
                self.states.reverse()
                self.totalCost = pastCost
                # if self.verbose >= 1:
                    #print "numStatesExplored = %d" % self.numStatesExplored
                    #print "totalCost = %s" % self.totalCost
                    ##print "actions = %s" % self.actions
                    ##print "states = %s" % self.states
                return self.states, self.exploredStates, self.frontierStates

            # Expand from |state| to new successor states,
            # updating the frontier with each newState.
            for action, newState, cost, h in problem.succAndCost(state):
                self.frontierStates.append([state, newState])
                #if self.verbose >= 3:
                    #print "  Action %s => %s with cost %s + %s" % (action, newState, pastCost, cost)
                #if frontier.update(newState, h): 
                if frontier.update(newState, pastCost + cost, h):
                    # Found better way to go to |newState|, update backpointer.
                    backpointers[newState] = (action, state)
            #print(time.time() - start_time)
        if self.verbose >= 1:
            print("No path found")

# Data structure for supporting uniform cost search.
class PriorityQueue:
    def  __init__(self):
        self.DONE = -100000
        self.heap = []
        self.priorities = {}  # Map from state to priority
        self.heuristics = {}

    # Insert |state| into the heap with priority |newPriority| if
    # |state| isn't in the heap or |newPriority| is smaller than the existing
    # priority.
    # Return whether the priority queue was updated.
    def update(self, state, newPriority, heuristic):
        self.heuristics[state] = heuristic
        oldPriority = self.priorities.get(state)
        if oldPriority == None or newPriority < oldPriority:
            self.priorities[state] = newPriority
            heapq.heappush(self.heap, (newPriority + self.heuristics[state], state))
            return True
        return False

    # Returns (state with minimum priority, priority)
    # or (None, None) if the priority queue is empty.
    def removeMin(self):
        while len(self.heap) > 0:
            priority, state = heapq.heappop(self.heap)
            pastCost = self.priorities[state]
            if self.priorities[state] == self.DONE: continue  # Outdated priority, skip
            self.priorities[state] = self.DONE
            return (state, pastCost, priority)
        return (None, None, None) # Nothing left...
