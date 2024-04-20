import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from modules import *
from rrt import nearest, sample_state

## With obstacles
def connect1(x_start, x_target, obstacles):
    max_dist = 0.3
    
    x_start = tuple_to_state(x_start)
    x_target = tuple_to_state(x_target)
    
    dist = distance_points(x_start, x_target)

    xnew = (x_target - x_start) / dist * max_dist + x_start
    xnew_tuple = state_to_tuple(xnew)

    if collide_obstacles(xnew_tuple, obstacles):
        return True
    else:
        return xnew    

def valid_state1(x, obstacles):
    if isinstance(x, bool):
        if x:
            return False
    if (np.abs(x[1:]) > 5.).any() or collide_obstacles(x, obstacles):
        return False
    return True

# RRT algorithm with obstacles
def RRT_obstacles(x_start:np.ndarray, x_goal:np.ndarray, obstacles:list, max_iters:int = 1000)->tuple:
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
    for i in range(max_iters):
        sample = sample_state(x_start)
        nearest_state = nearest(sample ,tree)
        x_new = connect1(nearest_state, sample, obstacles)
        if valid_state1(x_new, obstacles):
            # makin x_new a child of nearest_state
            tree[nearest_state].append(x_new) 
            # add a node to the tree
            tree[state_to_tuple(x_new)] = [] 
            if distance_points(x_new, x_goal) <= 0.1:
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
        collisions.append(distance_points(x, obstacle[:2]) <= obstacle[2])
    if any(collisions):
        return True
    else:
        return False

