import numpy as np
from modules import *
from rrt import nearest, sample_state
from modeling import simulate

r = .1
d = .25
rectangle_width = 4*d
rectangle_height = 2*d


def optimal_control(x_start:tuple, x_target:tuple, max_ites:int = 1000)->np.ndarray:
    """
    Connect the start state to the target state via an optimized path

    Args:
        x_start: start state
        x_target: target state
        K: number of iterations
    Returns:
        new state
    """
    # Optimization routine (slow)
    x_start = tuple_to_state(x_start)
    x_target = tuple_to_state(x_target)
    min_dist = np.inf

    def sample_control()->np.ndarray:
        """Sample a control input"""
        return np.random.uniform(-0.5, 0.5, (2, 1))
    
    for _ in range(max_ites):
        u = sample_control()
        states = simulate(x_start, u, 0.1, 4., "rk")
        dist = distance_points(state_to_tuple(states[-1])[1:], x_target[1:])
        if dist < min_dist:
            min_dist = dist
            best_state = states[-1]
    
    return best_state

def linear_path(x_start:tuple, x_target:tuple)->np.ndarray:
    """
    Connect the start state to the target state via a linear path

    Args:
        x_start: start state
        x_target: target state
    Returns:
        new state
    """
    max_dist = 0.3
    
    x_start = tuple_to_state(x_start)
    x_target = tuple_to_state(x_target)
    
    dist = distance_points(x_start, x_target)

    best_state = (x_target - x_start) / dist * max_dist + x_start

    return best_state

def connect1(x_start:tuple, x_target:tuple, opt:bool, obstacles:list)->np.ndarray:
    """
    Connect the start state to the target state via a linear path or an optimized path
    while avoiding obstacles

    Args:
        x_start: start state
        x_target: target state
    Returns:
        new state
    """
    best_state = None
    if opt:
        # Optimization routine (slow)
        best_state = optimal_control(x_start, x_target, 100)
    else:
        # Linear path
        best_state = linear_path(x_start, x_target)

    best_state_tuple = state_to_tuple(best_state)
    if collide_obstacles(best_state_tuple, obstacles):
        return True
    else:
        return best_state    

def valid_state1(x:np.ndarray, obstacles:list)->bool:
    """
    Check if the state is valid with the addition of obstacles

    Args:
        x: state
    Returns:
        True if the state is valid, False otherwise
    """
    if isinstance(x, bool):
        if x:
            return False
    if (np.abs(x[1:]) > 5.).any() or collide_obstacles(x, obstacles):
        return False
    return True

# RRT algorithm with obstacles
def RRT_obstacles(x_start:np.ndarray, x_goal:np.ndarray, opt:bool, obstacles:list, max_dist:float, max_iters:int = 1000)->tuple:
    """
    Rapidly-exploring Random Tree algorithm with obstacles

    Args:
        x_start: start state
        x_goal: goal state
        obstacles: list of obstacles
        max_iters: maximum number of iterations
    Returns:
        True if a path from the start to the goal was found, False otherwise
        tree: tree of states
    """
    tree = {}
    tree[state_to_tuple(x_start)] = []
    for _ in range(max_iters):
        sample = sample_state(x_start)
        nearest_state = nearest(sample ,tree)
        x_new = connect1(nearest_state, sample, opt, obstacles)
        if valid_state1(x_new, obstacles):
            # makin x_new a child of nearest_state
            tree[nearest_state].append(x_new) 
            # add a node to the tree
            tree[state_to_tuple(x_new)] = [] 
            if distance_points(x_new, x_goal) <= max_dist:
                return True, tree
    
    return False, tree

def collide_obstacles(x:tuple, obstacles:list)->bool:
    """
    Check if the state collides with the obstacles (only circles are supported for now)

    Args:
        x: state
        obstacles: list of obstacles
    Returns:
        True if the state collides with the obstacles, False otherwise
    """
    collisions = []

    for obstacle in obstacles:
        if len(x) > 2:
            point1 = x[1:]
            point2 = x[1:] + np.array([0., 2*d])
            point3 = x[1:] + np.array([4*d, 2*d])
            point4 = x[1:] + np.array([4*d, 0])

            col1 = distance_points(point1, obstacle[:2]) <= obstacle[2]
            col2 = distance_points(point2, obstacle[:2]) <= obstacle[2]
            col3 = distance_points(point3, obstacle[:2]) <= obstacle[2]
            col4 = distance_points(point4, obstacle[:2]) <= obstacle[2]

            collisions.append(col1 or col2 or col3 or col4)
        else:
            collisions.append(distance_points(x, obstacle[:2]) <= obstacle[2])
    if any(collisions):
        return True
    else:
        return False

