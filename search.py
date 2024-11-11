import util
from util import *
class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).
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
    """Buscar los nodos mas profundos del árbol primero."""
    stack=util.Stack()
    visited=[]
    startNode=(problem.getStartState(),[])        
    stack.push(startNode)
    while not stack.isEmpty():
        popped=stack.pop()
        location=popped[0]
        path=popped[1]
        if location not in visited:               
            visited.append(location)              
            if problem.isGoalState(location):
                return path
            successors=problem.getSuccessors(location)
            for suc in list(successors):
                if suc[0] not in visited:
                    stack.push((suc[0],path+[suc[1]]))
    return []
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Buscar primero los nodos más superficiales en el árbol de búsqueda."""
    queue=util.Queue()
    visited=[]                                                       
    startNode=(problem.getStartState(),[])                           
    queue.push(startNode)
    while not queue.isEmpty():
        popped=queue.pop()
        location=popped[0]
        path=popped[1]
        if location not in visited:
            visited.append(location)
            if problem.isGoalState(location):
                return path
            successors=problem.getSuccessors(location)
            for suc in list(successors):
                if suc[0] not in visited:
                    queue.push((suc[0],path + [suc[1]]))
    return []
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Buscamos el nodo que tenga menor costo total primero."""
    Pr_q = util.PriorityQueue()
    visited = {}
    state = problem.getStartState()
    nd = {
        "pred": None,  
        "act": None,   
        "state": state,
        "cost": 0
    }
    Pr_q.push(nd, nd["cost"])

    while not Pr_q.isEmpty():
        nd = Pr_q.pop()
        state = nd["state"]
        cost = nd["cost"]

        if state in visited:
            continue
        visited[state] = True
        if problem.isGoalState(state):
            break
        for suc in problem.getSuccessors(state):
            if suc[0] not in visited:
                new_nd = {
                    "pred": nd,
                    "state": suc[0],
                    "act": suc[1],
                    "cost": suc[2] + cost
                }
                Pr_q.push(new_nd, new_nd["cost"])

    # Reconstrucción del camino de acciones desde el nodo objetivo hasta el inicio
    actions = []
    while "act" in nd and nd["act"] is not None:
        actions.insert(0, nd["act"])
        nd = nd["pred"]
    return actions


def nullHeuristic(state, problem=None):
    """Funcion heurística que estima el costo del estado actual al mas cercano al objetivo. """
    return 0

def evalFunction(problem: SearchProblem, state, actions, heuristicFunction):
    """ Evaluates each state by its path cost + the heuristic cost. """
    return problem.getCostOfActions(actions) + heuristicFunction(state, problem)

def aStarSearch(problem, heuristic=nullHeuristic):
    """Buscar el nodo que tenga el costo total más bajo, sumando el costo acumulado y la heurística."""
    Pr_q = util.PriorityQueue()
    visited = {}

    state = problem.getStartState()
    nd = {
        "pred": None,
        "act": None,
        "state": state,
        "cost": 0,
        "eq": heuristic(state, problem)
    }
    Pr_q.push(nd, nd["cost"] + nd["eq"])

    while not Pr_q.isEmpty():
        nd = Pr_q.pop()
        state = nd["state"]
        cost = nd["cost"]

        if state in visited:
            continue
        visited[state] = True
        if problem.isGoalState(state):
            break
        for suc in problem.getSuccessors(state):
            if suc[0] not in visited:
                new_nd = {
                    "pred": nd,
                    "state": suc[0],
                    "act": suc[1],
                    "cost": suc[2] + cost,
                    "eq": heuristic(suc[0], problem)
                }
                Pr_q.push(new_nd, new_nd["cost"] + new_nd["eq"])
    actions = []
    while nd["act"] is not None:
        actions.insert(0, nd["act"])
        nd = nd["pred"]
    return actions


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch