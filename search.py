# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    from util import Stack

    dataStructure = Stack()     # use stack as a data structure for depthFirstSearch
    startState = problem.getStartState()
    dataStructure.push((startState, [], 0))     # push the startState onto the stack
    node = dataStructure.pop()      # process the startState
    visited = []
    visited.append(node[0])     # put the startState in the list of visited nodes
    actions = []     # initialize a list to keep track of the list of actions to take to reach the goal state
    actions = node[1]
    cost = node[2]

    while not problem.isGoalState(node[0]):     # Loop till we don't reach the goal state
        successors = problem.getSuccessors(node[0])
        for successor in successors:
            if (not successor[0] in visited) or (problem.isGoalState(successor[0])):
                # push the node to the stack if it has not been visited
                dataStructure.push((successor[0], actions + [successor[1]], cost + successor[2]))
                visited.append(successor[0])     # put the node to visited nodes after it has been processed
        node = dataStructure.pop()
        actions = node[1]
        cost = node[2]
    return actions

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    from util import Queue

    startState = problem.getStartState()
    dataStructure = Queue()     # Use queue as a data structure for breadthFirstSearch
    dataStructure.push((problem.getStartState(), [], 0))    # push the startState on the queue
    node = dataStructure.pop()      # process the startState
    visited = []
    visited.append(node[0])     # put the startState in the list of visited nodes
    actions = []
    actions = node[1]
    cost = node[2]

    while not problem.isGoalState(node[0]):     # Loop till we don't reach the goal state
        successors = problem.getSuccessors(node[0])
        for successor in successors:
            if not successor[0] in visited:
                # If node has not been visited, then push the node on to the queue
                dataStructure.push((successor[0], actions + [successor[1]], cost + successor[2]))
                visited.append(successor[0])  # put the node to visited nodes after it has been processed
        node = dataStructure.pop()
        actions = node[1]
        cost = node[2]

    return actions

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    from util import PriorityQueue

    dataStructure = PriorityQueue()     # Use priorityQueue as the data structure for uniformCostSearch
    dataStructure.push((problem.getStartState(), [], 0), 0)     # push the startState on the priorityQueue
    node = dataStructure.pop()
    actions = []
    actions = node[1]
    cost = node[2]
    visited = []
    visited.append((node[0], node[2]))

    while not problem.isGoalState(node[0]):     # Loop till we don't reach the goal state
        successors = problem.getSuccessors(node[0])
        for successor in successors:
            tCost = cost + successor[2]     # total cost to reach the node from starting node
            vFlag = False  # keep a boolean flag to keep track of the visited node
            for (vState, vCost) in visited:
                # If the node is already visited and its cost is lower than the current path cost, do nothing
                # Else, add the node to the visited list with updated cost
                if (vState == successor[0] and tCost>=vCost):
                    vFlag = True
                    break
            if not vFlag:
                dataStructure.push((successor[0], actions + [successor[1]], cost+successor[2]), cost+successor[2])
                visited.append((successor[0],cost+successor[2]))
        node = dataStructure.pop()
        actions = node[1]
        cost = node[2]
    return actions

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    from util import PriorityQueue

    dataStructure = PriorityQueue()     # Use priorityQueue as a data structure for A* search
    # Add the startState to the priorityQueue with heuristic added to the cost
    dataStructure.push((problem.getStartState(), [], 0),0+heuristic(problem.getStartState(), problem))
    node = dataStructure.pop()      # Process the startState
    actions = []
    actions = node[1]
    cost = node[2]
    visited = []
    # Add the startState to the list of visited nodes
    visited.append((node[0], cost + heuristic(problem.getStartState(), problem)))

    while not problem.isGoalState(node[0]):     # Loop till we don't reach to the goal state
        successors = problem.getSuccessors(node[0])
        for successor in successors:
            vFlag = False     # Initialize a boolean flag to keep track of the visited node
            tCost = cost + successor[2]     # Calculate the total cost from the startState to the successor
            for (vState, vCost) in visited:
                if (successor[0] == vState) and (tCost >= vCost):
                    vFlag = True
                    break
            if not vFlag:
                dataStructure.push((successor[0], actions + [successor[1]], cost + successor[2]),
                            cost + successor[2] + heuristic(successor[0], problem))
                visited.append((successor[0], cost + successor[2]))
        node = dataStructure.pop()
        actions = node[1]
        cost = node[2]
    return actions

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
