from collections import deque
import numpy as np
from search_problems import Node, GraphSearchProblem

def breadth_first_search(problem):
    """
    Implement a simple breadth-first search algorithm that takes instances of SimpleSearchProblem (or its derived
    classes) and provides a valid and optimal path from the initial state to the goal state. Useful for testing your
    bidirectional and A* search algorithms.

    :param problem: instance of SimpleSearchProblem
    :return: path: a list of states (ints) describing the path from problem.init_state to problem.goal_state[0]
             num_nodes_expanded: number of nodes expanded by your search
             max_frontier_size: maximum frontier size during search
    """
    queue = [] #make a nested list that contains information of prev nodes in a path
    path = []
    checked = [] #list to keep track of visited nodes
    s = problem.init_state
    g = problem.goal_states
    queue.append([s]) 
    n = 0 #keep count of max frontier size
    while queue:
        path = queue.pop(0) 
        u = int(path[-1]) 
        #get the last node from most recent path
    
        if u not in checked: 
            v = problem.neighbours[u] 
            #explore the neighbours of such node
            
            for a in v:
                i = int(a)
                new_path = list(path)
                new_path.append(i)
                queue.append(new_path) 
                #append the entire path including new node to queue
                
                if len(queue) > n:
                    n = len(queue) 
                    #check if frontier size changed
                
                if i == int(g[0]): 
                    #reached goal state, a path is found
                    path.append(i)
                    #add goal state to path
                    max_frontier_size = n
                    num_nodes_expanded = len(checked)
                    #terminate program from here
                    return path, num_nodes_expanded, max_frontier_size
                
            #finished checking all neiboughrs add current node to checked   
            checked.append(int(u)) 
    
    num_nodes_expanded = len(checked)    
    return path, num_nodes_expanded, max_frontier_size
    

if __name__ == '__main__':
    # Simple example
    goal_states = [0]
    init_state = 9
    V = np.arange(0, 10)
    E = np.array([[0, 1],
                  [1, 2],
                  [2, 3],
                  [3, 4],
                  [4, 5],
                  [5, 6],
                  [6, 7],
                  [7, 8],
                  [8, 9],
                  [0, 6],
                  [1, 7],
                  [2, 5],
                  [9, 4]])
    problem = GraphSearchProblem(goal_states, init_state, V, E)
    path, num_nodes_expanded, max_frontier_size = breadth_first_search(problem)
    correct = problem.check_graph_solution(path)
    print("Solution is correct: {:}".format(correct))
    print(path)

    # Use stanford_large_network_facebook_combined.txt to make your own test instances
    E = np.loadtxt('stanford_large_network_facebook_combined.txt', dtype=int)
    V = np.unique(E)
    goal_states = [349]
    init_state = 0
    problem = GraphSearchProblem(goal_states, init_state, V, E)
    path, num_nodes_expanded, max_frontier_size = breadth_first_search(problem)
    correct = problem.check_graph_solution(path)
    print("Solution is correct: {:}".format(correct))
    print(path)
