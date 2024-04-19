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
def RRT_obstacles(x_start:np.array, x_goal:np.array, obstacles:list, max_iters:int = 1000)->tuple:
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
    Check if the state collides with the obstacles (only circles are supported)

    Args:
        x: state
        obstacles: list of obstacles
    Returns:
        True if the state collides with the obstacles, False otherwise
    """
    collision1 = distance_points(x, obstacles[0][:2]) <= obstacles[0][2]
    collision2 = distance_points(x, obstacles[1][:2]) <= obstacles[1][2]
    return collision1 or collision2

def visualize2D_with_sobstacles(tree:dict, obstacles:list)->None:
    """
    Visualize the tree of states with obstacles
    
    Args:
        tree: tree of states
        obstacles: list of obstacles
    Returns: None
    """
    
    fig = plt.figure()
    ax = fig.add_subplot(111)

    for s in tree:
        ax.plot(s[0], s[1], '.', zorder=2)

        for c in tree[s]:
            ax.plot([s[0], c[0, 0]], [s[1], c[1, 0]], zorder=1)

    for o in obstacles:
        ax.add_patch(Circle([o[0], o[1]], radius=o[2], fill=False, zorder=3))

    plt.ylim(-5., 5.)
    plt.xlim(-5., 5.)
    plt.show()