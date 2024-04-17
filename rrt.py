import numpy as np
from modules import *



def sample_state()->tuple:
    """
    Sample a random state and return it in tuple form
    Returns:
        random state
    """
    state = np.array([[np.random.uniform(-5., 5.), np.random.uniform(-5., 5.)]])
    return state_to_tuple(state)
   

# Distance between two points
def distance_points(point1:tuple, point2:tuple)->float:
    """
    Calculate the distance between two points
    Args:
        point1: first point
        point2: second point
    Returns:
        distance between the two points
    """
    return np.sqrt( (point1[0] - point2[0])**2 +  (point1[1] - point2[1])**2 )
    

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
    

def connect(x_start:tuple, x_target:tuple)->np.array:
    max_dist = 0.3
    
    x_start = tuple_to_state(x_start)
    x_target = tuple_to_state(x_target)
    
    dist = distance_points(x_start, x_target)

    return (x_target - x_start) / dist * max_dist + x_start
    
def valid_state(x:np.array)->bool:
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

# RRT algorithm
def RRT(x_start, x_goal, max_iters = 1000):
    tree = {}
    tree[state_to_tuple(x_start)] = []
    for i in range(max_iters):
        sample = sample_state()
        nearest_state = nearest(sample ,tree)
        x_new = connect(nearest_state, sample)
        if valid_state(x_new):
            # makin x_new a child of nearest_state
            tree[nearest_state].append(x_new) 
            # add a node to the tree
            tree[state_to_tuple(x_new)] = [] 
            if distance_points(x_new, x_goal) <= 0.1:
                return True, tree
    
    return False, tree