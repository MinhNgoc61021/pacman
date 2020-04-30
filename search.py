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
import enum
from game import Directions

s = Directions.SOUTH
w = Directions.WEST
e = Directions.EAST
n = Directions.NORTH

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
    import util

    def depthFirstSearchUtils(problem, action_list, visited_list, stack, state, direction, actions):
        visited_list.append(state)
        actions.append(direction)
        if problem.isGoalState(state):
            action_list.append([action for action in actions if action != ''])
        else:
            for successor, direction, cost in problem.getSuccessors(state):
                if successor not in visited_list:
                    depthFirstSearchUtils(problem, action_list, visited_list, stack, successor, direction, actions)

        actions.pop()
        visited_list.remove(state)
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    
    """
    "*** YOUR CODE HERE ***"
    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    stack = util.Stack()
    init_state = problem.getStartState()
    action_list = []

    depthFirstSearchUtils(problem, action_list, [], stack, init_state, "", [])

    print(action_list)
    action_cost = {}
    for index, action in enumerate(action_list):
        action_cost[index] = problem.getCostOfActions(action)
    min_index = min(action_cost, key=action_cost.get)
    return action_list[min_index]


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    import util

    def breadthFirstSearchUtils(problem, action_list, visited_list, queue, current_state, direction, actions):
        if queue.isEmpty():
            visited_list.add(current_state)
            queue.push(current_state)
            actions.append(direction)

        while queue.isEmpty() is not True:
            state = queue.pop()
            if problem.isGoalState(state):
                action_list.append([action for action in actions if action != ''])
            else:
                for successor, direction, cost in problem.getSuccessors(state):
                    if (successor not in visited_list) and (successor not in queue.list):
                        breadthFirstSearchUtils(problem, action_list, visited_list, queue, successor, direction , actions)
        actions.pop()
    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState())) # [(successor, action, stepCost), ...]

    queue = util.Queue()
    init_state = problem.getStartState()
    action_list = []
    visited_list = set()

    breadthFirstSearchUtils(problem, action_list, visited_list, queue, init_state, '', [])

    # print(action_list)
    action_cost = {}
    for index, action in enumerate(action_list):
        action_cost[index] = problem.getCostOfActions(action)
    min_index = min(action_cost, key=action_cost.get)
    return action_list[min_index]


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
