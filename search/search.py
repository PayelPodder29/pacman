"""
search.py
"""
import util

class SearchProblem:
    """
    This class outlines the structure of a search problem.
    """

    def getStartState(self):
        util.raiseNotDefined()

    def isGoalState(self, state):
        util.raiseNotDefined()

    def getSuccessors(self, state):
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        util.raiseNotDefined()

def tinyMazeSearch(problem):
    """Hardcoded solution for tinyMaze."""
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """Search the deepest nodes in the search tree first."""
    from util import Stack

    frontier = Stack()
    visited = set()
    frontier.push((problem.getStartState(), [], 0))  # (state, actions, cost)

    while not frontier.isEmpty():
        state, actions, cost = frontier.pop()

        if state in visited:
            continue
        visited.add(state)

        if problem.isGoalState(state):
            return actions

        for nextState, action, stepCost in problem.getSuccessors(state):
            if nextState not in visited:
                frontier.push((nextState, actions + [action], cost + stepCost))
        
    return []  # Return empty list if no solution is found

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    from util import Queue

    frontier = Queue()
    visited = set()
    frontier.push((problem.getStartState(), []))  # (state, actions)

    while not frontier.isEmpty():
        state, actions = frontier.pop()

        if state in visited:
            continue
        visited.add(state)

        if problem.isGoalState(state):
            return actions

        for nextState, action, stepCost in problem.getSuccessors(state):
            if nextState not in visited:
                frontier.push((nextState, actions + [action]))

    return []

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    from util import PriorityQueue

    frontier = PriorityQueue()
    visited = set()
    frontier.push((problem.getStartState(), [], 0), 0)  # (state, actions, cost), priority

    while not frontier.isEmpty():
        state, actions, cost = frontier.pop()

        if state in visited:
            continue
        visited.add(state)

        if problem.isGoalState(state):
            return actions

        for nextState, action, stepCost in problem.getSuccessors(state):
            if nextState not in visited:
                newCost = cost + stepCost
                frontier.push((nextState, actions + [action], newCost), newCost)

    return []

def nullHeuristic(state, problem=None):
    """A trivial heuristic function."""
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node with the lowest cost + heuristic."""
    from util import PriorityQueue

    frontier = PriorityQueue()
    visited = set()
    startState = problem.getStartState()
    frontier.push((startState, [], 0), 0)  # (state, actions, cost), priority

    while not frontier.isEmpty():
        state, actions, cost = frontier.pop()


        if state in visited:
            continue
        visited.add(state)

        if problem.isGoalState(state):
            return actions

        for nextState, action, stepCost in problem.getSuccessors(state):
            if nextState not in visited:
                newCost = cost + stepCost
                priority = newCost + heuristic(nextState, problem)
                frontier.push((nextState, actions + [action], newCost), priority)

    return []

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
ucs = uniformCostSearch
astar = aStarSearch
