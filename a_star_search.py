import queue
import numpy as np
from search_problems import Node, GridSearchProblem, get_random_grid_problem


def a_star_search(problem):
    """
    Uses the A* algorithm to solve an instance of GridSearchProblem. Use the methods of GridSearchProblem along with
    structures and functions from the allowed imports (see above) to implement A*.

    :param problem: an instance of GridSearchProblem to solve
    :return: path: a list of states (ints) describing the path from problem.init_state to problem.goal_state[0]
             num_nodes_expanded: number of nodes expanded by your search
             max_frontier_size: maximum frontier size during search
    """
    ####
    #   COMPLETE THIS CODE
    # f = g + h
    # priority queue -> explore least cost first
    ####
    
    s = problem.init_state
    goal = problem.goal_states[0]
    p_q = queue.PriorityQueue()
    #(f(n),n)
    num_nodes_expanded = 0
    max_frontier_size = 0
    path = []
    checked = set()
    
    #initialize priority queue
    checked.add(s)
    f_start = problem.manhattan_heuristic(s, s) + problem.heuristic(s)
    #f = g+h
    start = Node(None, s, (s, s),0)
    #starting node
    p_q.put((f_start, start))
    #(f(n),n)
    
    while not p_q.empty() :
        
        curr = p_q.get()
        #this will be a node
        #(cost,node)
        #curr[0] = cost, curr[1] = node
        
        if curr[1].state == goal:
            #goal has been reached 
            #terminate while loop
            path = problem.trace_path(curr[1],s)
            #trace path using trace_path
            num_nodes_expanded = len(checked)
            return path, num_nodes_expanded, max_frontier_size

        for v in problem.get_actions(curr[1].state):
            #action_list[(curr,child)]
            #explore all children of current 
            child = problem.get_child_node(curr[1],v)
            #get_child_node(parent,action)
            #return a child node(parent_node, child_state, action, child_path_cost)
                                     
            if child.state not in checked:
                #if the child is not visited
                #get its cost and add it to the priority queue
                f = child.path_cost + problem.heuristic(child.state)
                p_q.put((f, child))
                #update visited list
                checked.add(child.state)
    
    path = []
    num_nodes_expanded = len(checked)
    #this will be returned when no path is found
    return path, num_nodes_expanded, max_frontier_size
    

def search_phase_transition():
    """
    Simply fill in the prob. of occupancy values for the 'phase transition' and peak nodes expanded within 0.05. You do
    NOT need to submit your code that determines the values here: that should be computed on your own machine. Simply
    fill in the values!

    :return: tuple containing (transition_start_probability, transition_end_probability, peak_probability)
    """
    ####
    #   REPLACE THESE VALUES
    ####
    transition_start_probability = 0.3
    transition_end_probability = 0.45
    peak_nodes_expanded_probability = 0.35
    return transition_start_probability, transition_end_probability, peak_nodes_expanded_probability


if __name__ == '__main__':
    # Test your code here!
    # Create a random instance of GridSearchProblem
    p_occ = 0.25
    M = 10
    N = 10
    problem = get_random_grid_problem(p_occ, M, N)
    # Solve it
    path, num_nodes_expanded, max_frontier_size = a_star_search(problem)
    # Check the result
    correct = problem.check_solution(path)
    print("Solution is correct: {:}".format(correct))
    # Plot the result
    problem.plot_solution(path)

    # Experiment and compare with BFS