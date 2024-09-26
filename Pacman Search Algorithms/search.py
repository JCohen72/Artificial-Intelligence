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
    Search the deepest nodes in the search tree first using Depth-First Search (DFS).
    Returns a list of actions that reaches the goal.
    """
    fringe = util.Stack()  # Stack for DFS
    fringe.push((problem.getStartState(), []))
    visited = set()  # Set of visited states

    while not fringe.isEmpty():
        state, path = fringe.pop()
        if problem.isGoalState(state):
            return path
        if state not in visited:
            visited.add(state)
            # Push all successors onto the stack
            for successor, action, _ in problem.getSuccessors(state):
                fringe.push((successor, path + [action]))
    return []  # Return empty path if no solution found

def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first using Breadth-First Search (BFS).
    """
    fringe = util.Queue()  # Queue for BFS
    fringe.push((problem.getStartState(), []))
    visited = set([problem.getStartState()])  # Set of visited states

    while not fringe.isEmpty():
        state, path = fringe.pop()
        if problem.isGoalState(state):
            return path
        # Enqueue successors
        for successor, action, _ in problem.getSuccessors(state):
            if successor not in visited:
                visited.add(successor)
                fringe.push((successor, path + [action]))
    return []  # Return empty path if no solution found

def uniformCostSearch(problem):
    """
    Search the node of least total cost first using Uniform Cost Search (UCS).
    """
    fringe = util.PriorityQueue()
    fringe.push((problem.getStartState(), []), 0)
    visited = {}  # Map of visited states to their lowest cost

    while not fringe.isEmpty():
        state, path = fringe.pop()
        cost = problem.getCostOfActions(path)
        if state in visited and visited[state] <= cost:
            continue
        visited[state] = cost
        if problem.isGoalState(state):
            return path
        # Enqueue successors with updated cost
        for successor, action, stepCost in problem.getSuccessors(state):
            newPath = path + [action]
            newCost = problem.getCostOfActions(newPath)
            fringe.push((successor, newPath), newCost)
    return []  # Return empty path if no solution found

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """
    Search the node with the lowest combined cost and heuristic first using A* Search.
    """
    fringe = util.PriorityQueue()
    startState = problem.getStartState()
    fringe.push((startState, []), heuristic(startState, problem))
    visited = {}  # Map of visited states to their lowest cost

    while not fringe.isEmpty():
        state, path = fringe.pop()
        cost = problem.getCostOfActions(path)
        if state in visited and visited[state] <= cost:
            continue
        visited[state] = cost
        if problem.isGoalState(state):
            return path
        # Enqueue successors with updated cost and heuristic
        for successor, action, stepCost in problem.getSuccessors(state):
            newPath = path + [action]
            newCost = problem.getCostOfActions(newPath)
            totalCost = newCost + heuristic(successor, problem)
            fringe.push((successor, newPath), totalCost)
    return []  # Return empty path if no solution found

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
