import numpy as np
from modules import distance_points, state_to_tuple, optimal_control, linear_path
from rrt_with_obstacles import collide_obstacles

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

def connect(x_start:tuple, x_target:tuple, opt:bool, obstacles:list=[])->np.ndarray:
    """
    Connect the start state to the target state via a linear path or an optimized path
    while avoiding obstacles

    Args:
        x_start: start state
        x_target: target state
        opt: optimization flag
        obstacles: list of obstacles
    Returns:
        best_state: new state
        states: trajectory from start to target
    """

    best_state = None
    states = None
    if opt:
        # Optimization routine (slow)
        best_state, states = optimal_control(x_start, x_target, 100)
    else:
        # Linear path
        best_state = linear_path(x_start, x_target)

    if obstacles:
        best_state_tuple = state_to_tuple(best_state)
        if collide_obstacles(best_state_tuple, obstacles):
            return True, states
        else:
            return best_state, states
    else:
        return best_state, states

def valid_state(x:np.ndarray, obstacles:list=[])->bool:
    """
    Check if the state is valid with the addition of obstacles

    Args:
        x: state
        obstacles: list of obstacles
    Returns:
        True if the state is valid, False otherwise
    """
    if obstacles:
        if isinstance(x, bool):
            if x:
                return False
        if (np.abs(x[1:]) > 5.).any() or collide_obstacles(x, obstacles):
            return False
        return True
    else:
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

# RRT algorithm
def RRT(x_start:np.ndarray, x_goal:np.ndarray, opt:bool, max_dist:float, max_iters:int = 1000)->tuple:
    """
    Rapidly-exploring Random Tree algorithm

    Args:
        x_start: start state
        x_goal: goal state
        opt: optimization flag
        max_dist: maximum distance between end state and goal state
        max_iters: maximum number of iterations
    Returns:
        True if a path from the start to the goal was found, False otherwise
        tree: tree of states
    """

    tree = {}
    tree[state_to_tuple(x_start)] = []
    trajectories = []
    for _ in range(max_iters):
        sample = sample_state(x_start)
        nearest_state = nearest(sample ,tree)
        x_new, trajectory = connect(nearest_state, sample, opt)
        trajectories.append(trajectory)
        if valid_state(x_new):
            # makin x_new a child of nearest_state
            tree[nearest_state].append(x_new) 
            # add a node to the tree
            tree[state_to_tuple(x_new)] = [] 
            
            if len(sample) > 2:
                dist = distance_points(x_new[1:], x_goal[1:])
            else:
                dist = distance_points(x_new, x_goal)

            if dist <= max_dist:
                    return True, tree, trajectories
    
    return False, tree, trajectories







