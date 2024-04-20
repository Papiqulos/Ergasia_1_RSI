import numpy as np
from modules import *
from modeling import simulate

def sample_state(state_template:np.ndarray)->tuple:
    """
    Sample a random state and return it in tuple form

    Args:
        state_template: the type of state to be randomly generated

    Returns:
        random state
    """
    state = np.array([[np.random.uniform(-5., 5.) for _ in range(len(state_template[:,0]))]]).T
    return state_to_tuple(state)
 
def valid_state(x:np.ndarray)->bool:
    """
    Check if the state is valid

    Args:
        x: state
    Returns:
        True if the state is valid, False otherwise
    """
    
    if (np.abs(x[1:]) > 5.).any():
        return False
    return True 

def nearest(x_sample:tuple, tree:dict)->tuple:
    """
    Find the nearest state in the tree to the sample state

    Args:
        x_sample: sample state
        tree: tree of states
    Returns:
        nearest state to the sample state
    """
    min_distance = np.inf
    nearest_state = None

    for state in tree:
        distance = distance_points(state, x_sample)
        if distance < min_distance:
            min_distance = distance
            nearest_state = state
    
    return nearest_state
    
def connect(x_start:tuple, x_target:tuple, opt:bool)->np.ndarray:
    """
    Connect the start state to the target state via a linear path or an optimized path

    Args:
        x_start: start state
        x_target: target state
    Returns:
        new state
    """
    best_state = None
    if opt:
        # Optimization routine (slow)
        x_start = tuple_to_state(x_start)
        x_target = tuple_to_state(x_target)
        min_dist = np.inf

        def sample_control():
            return np.random.uniform(-0.5, 0.5, (2, 1))
        
        K = 100
        for _ in range(K):
            u = sample_control()
            states = simulate(x_start, u, 0.1, 4., "rk")
            dist = distance_points(state_to_tuple(states[-1]), x_target)
            if dist < min_dist:
                min_dist = dist
                best_state = states[-1]
    else:
        ## Linear path
        max_dist = 0.3
        
        x_start = tuple_to_state(x_start)
        x_target = tuple_to_state(x_target)
        
        dist = distance_points(x_start, x_target)

        best_state = (x_target - x_start) / dist * max_dist + x_start

    return best_state

# RRT algorithm
def RRT(x_start:np.ndarray, x_goal:np.ndarray, opt:bool, max_dist:float, max_iters:int = 1000)->tuple:
    """
    Rapidly-exploring Random Tree algorithm

    Args:
        x_start: start state
        x_goal: goal state
        max_iters: maximum number of iterations
    Returns:
        True if a path from the start to the goal was found, False otherwise
        tree: tree of states
    """

    tree = {}
    tree[state_to_tuple(x_start)] = []
    for i in range(max_iters):
        sample = sample_state(x_start)
        nearest_state = nearest(sample ,tree)
        x_new = connect(nearest_state, sample, opt)
        if valid_state(x_new):
            # makin x_new a child of nearest_state
            tree[nearest_state].append(x_new) 
            # add a node to the tree
            tree[state_to_tuple(x_new)] = [] 
            if len(sample) > 2:
                if distance_points(x_new[1:], x_goal[1:]) <= max_dist:
                    return True, tree
            else:
                if distance_points(x_new, x_goal) <= max_dist:
                    return True, tree
    
    return False, tree







