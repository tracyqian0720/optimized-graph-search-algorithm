from collections import deque
import numpy as np
from search_problems import Node, GraphSearchProblem
from breadth_first_search import breadth_first_search

def bidirectional_search(problem):
    """
        Implement a bidirectional search algorithm that takes instances of SimpleSearchProblem (or its derived
        classes) and provides a valid and optimal path from the initial state to the goal state.

        :param problem: instance of SimpleSearchProblem
        :return: path: a list of states (ints) describing the path from problem.init_state to problem.goal_state[0]
                 num_nodes_expanded: number of nodes expanded by your search
                 max_frontier_size: maximum frontier size during search
        """
    
    
    s = problem.init_state
    g = problem.goal_states
    #make a nested list that contains information of prev nodes in a path
    #each nested list is a path that leads to a node whose value is the index of the nested list
    vert = problem.V
    f_queue = [[] for i in vert]
    b_queue = [[] for i in vert]
    path = []
    max_frontier_size = 0
    num_nodes_expanded = 0    
    #keep track of visited nodes
    f_checked = set()
    b_checked = set()
    #keep track of the frontier
    f_front = [s]
    b_front = [g[0]]
    n = 0
    
    f_queue[s].append(s)
    b_queue[g[0]].append(g[0])
    
    while f_front or b_front:     
        
        f_curr_front = []
        b_curr_front = []
        
        for f_u in f_front:
            f_adj = problem.get_actions(f_u)
            #get all the successor nodes of the next layer
            for f_v in f_adj:
                #explore each successor node individually
                if f_v[1] not in f_checked:
                    #check if such node has been visited
                    if f_v[1] not in f_curr_front:
                        #check if such node is a successor of one of the nodes already
                        f_curr_front.append(f_v[1])
                        #add the successor node to the temporary layer tracker
                        new_path = list(f_queue[f_u])
                        new_path.append(f_v[1])
                        f_queue[f_v[1]] = new_path
                        #update f_queue with the new node
                        f_checked.add(f_v[1])
                        #update visited list
                        
                        if len(f_curr_front) > n:
                            #check if frontier size changed
                            n = len(f_curr_front)
                        
                if f_v[1] in b_checked or f_v[1] in b_front:
                    #check if such node has been visited or is being visited
                    #in which case we can terminate our program
                    f_checked.add(f_v[1])
                    #update visited list
                    b_curr = b_queue[f_v[1]]
                    b_curr.pop()
                    b_curr.reverse()
                    #get the backward path leading to this node
                    #pop this node off so we don't repeat it twice
                    path = f_queue[f_v[1]] + b_curr
                    #final path
                    max_frontier_size = n
                    num_nodes_expanded = len(f_checked) + len(b_checked)  
                    #terminate program 
                    #print(path)
                    return path, num_nodes_expanded, max_frontier_size
            
                
            f_checked.add(f_u)
            #update parent node to visited list
        f_front = f_curr_front
        #move on to the next layer 
        
        #backward pass works the same way as forward pass
        for b_u in b_front:
            b_adj = problem.get_actions(b_u)
            for b_v in b_adj:
                if b_v[1] not in b_checked:
                    if b_v[1] not in b_curr_front:
                        b_curr_front.append(b_v[1])
                        new_path_b = list(b_queue[b_u])
                        new_path_b.append(b_v[1])
                        b_queue[b_v[1]] = new_path_b
                        b_checked.add(b_v[1])
                        
                        if len(b_curr_front) > n:
                            n = len(b_curr_front)
                            
                if b_v[1] in f_checked or b_v[1] in f_front:
                    b_checked.add(b_v[1])
                    f_curr = f_queue[b_v[1]]
                    f_curr.pop()
                    b_queue[b_v[1]].reverse()
                    path = f_curr + b_queue[b_v[1]]
                    max_frontier_size = n
                    num_nodes_expanded = len(f_checked) + len(b_checked)
                    #print(path)
                    return path, num_nodes_expanded, max_frontier_size
                
            b_checked.add(b_u)
        b_front = b_curr_front
    
    path = []
    num_nodes_expanded = len(checked)
    #this will be returned when no path is found    
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
    path, num_nodes_expanded, max_frontier_size = bidirectional_search(problem)
    correct = problem.check_graph_solution(path)
    print("Solution is correct: {:}".format(correct))
    print(path)
    
    
    # Use stanford_large_network_facebook_combined.txt to make your own test instances
    E = np.loadtxt('stanford_large_network_facebook_combined.txt', dtype=int)
    V = np.unique(E)
    goal_states = [349]
    init_state = 0
    problem = GraphSearchProblem(goal_states, init_state, V, E)
    path, num_nodes_expanded, max_frontier_size = bidirectional_search(problem)
    correct = problem.check_graph_solution(path)
    print("Solution is correct: {:}".format(correct))
    print(path)

    # Be sure to compare with breadth_first_search!
    