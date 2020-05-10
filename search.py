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
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    import util

    stack = util.Stack()
    init_state = problem.getStartState()
    action_list = []
    stack.push(init_state)

    visited = []

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
    queue = util.Queue()
    init_state = problem.getStartState()
    action_list = []
    queue.push((init_state, action_list))
    visited_list = []

    def breadthFirstSearchUtils(problem, visited_list, queue):
        while queue.isEmpty() is not True:
            state, action_list = queue.pop()
            visited_list.append(state)

            if problem.isGoalState(state):
                return action_list

            successors = problem.getSuccessors(state)
            for (successor, direction, cost) in successors:
                if (successor not in visited_list) and (successor not in (state for state in queue.list)):
                    if problem.isGoalState(state):
                        return action_list + [direction]
                    else:
                        new_action_list = action_list + [direction]
                        queue.push((successor, new_action_list))

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))  # [(successor, action, stepCost), ...]

    if problem.isGoalState(init_state):
        return []
    else:
        return breadthFirstSearchUtils(problem, visited_list, queue)

    # print(action_list)


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
    class Node:
        def __init__(self, parent, position, action, cost, g, h, f):
            self.parent = parent
            self.position = position
            self.action = action
            self.cost = cost

            self.g = g
            self.h = h
            self.f = f

        def __str__(self):
            return self.action
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    def in_close_list(item, list):
        for l in list:
            if item.position == l.position:
                return True
        return False

    def in_open_list(item, list):
        for l in list:
            if item.position == l.position and l.f <= item.f:
                return True
        return False

    init_state = Node(None, problem.getStartState(), None, 0, 0, 0, 0)  # parent, position, action, g, h, f
    open_list = []
    close_list = []

    open_list.append(init_state)
    while len(open_list) > 0:
        #print("open_list f:", ["g{}+h{}=f{}/{}/{}".format(node.g, node.h, node.f, node.action, node.position) for node in open_list])
        current_node = open_list[0]
        current_index = 0
        ###Tìm node có f bé nhất###
        for index, node in enumerate(open_list):
            if node.f < current_node.f:
                current_node = node
                current_index = index
        #print("current_index:", current_index, "//smallest f:","g{}+h{}=f{}/{}/{}".format(current_node.g,current_node.h,current_node.f,current_node.action,current_node.position))
        #print("current(min) node:", "g{}+h{}=f{}/{}/{}".format(current_node.g,current_node.h,current_node.f,current_node.action,current_node.position))

        ###Pop nốt đó khỏi open_list và thêm vào close_list###
        open_list.pop(current_index)
        close_list.append(current_node)

        #print("close_list f:", ["g{}+h{}=f{}/{}/{}".format(node.g,node.h,node.f,node.action,node.position) for node in close_list])

        ###Nếu node đang xét là goal thì dừng và lần ngược lại action đến start node###
        if problem.isGoalState(current_node.position):
            action_list = []
            current = current_node
            while current is not None:
                action_list.append(current.action)
                #print("acion: ", action_list)
                #print("g{}+h{}=f{}/{}/{}".format(current.g, current.h, current.f, current.action, current.position))
                current = current.parent
            return action_list[::-1][1:] #xóa start node có action là None

        ###Lấy các children của node###
        children = []
        for child in problem.getSuccessors(current_node.position):
            children.append(Node(parent=current_node,
                                 position=child[0],
                                 cost=child[2],
                                 action=child[1],
                                 g=0, h=0, f=0))
        #print("children:", ["g{}+h{}=f{}/{}/{}".format(child.g,child.h,child.f,child.action,child.position) for child in children])


        #print("CHILDREN LOOP")
        for index, child in enumerate(children):
            ###Nếu node không có trong close_list###
            if not in_close_list(child, close_list):
                child.g = current_node.g + child.cost
                child.h = heuristic(child.position, problem)
                child.f = child.g + child.h
                #print("update child no.{}: g{}+h{}=f{}/{}/{}".format(index, child.g, child.h, child.f, child.action, child.position))
                ###Nếu không thỏa mãn node có trong open_list và tồn tại node có f nhỏ hơn child đang xét###
                if not in_open_list(child, open_list):
                    #print("append child no.{} to open_list".format(index))
                    open_list.append(child)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
