import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from modules import *
from modeling import *

def sample_state(state_template:np.array)->tuple:
    """
    Sample a random state and return it in tuple form

    Args:
        state_template: the type of state to be randomly generated

    Returns:
        random state
    """
    state = np.array([[np.random.uniform(-5., 5.) for _ in range(len(state_template[:,0]))]]).T
    return state_to_tuple(state)
   
def distance_points(point1:tuple, point2:tuple)->float:
    """
    Calculate the distance between two points in tuple form

    Args:
        point1: first point
        point2: second point
    Returns:
        distance between the two points
    """
    point1 = tuple_to_state(point1)
    point2 = tuple_to_state(point2)
    distance = np.linalg.norm(point1 - point2)
    return distance

## Without obstacles  
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
    """
    Connect the start state to the target state via a linear path

    Args:
        x_start: start state
        x_target: target state
    Returns:
        new state
    """
    ## Linear path
    # max_dist = 0.3
    
    # x_start = tuple_to_state(x_start)
    # x_target = tuple_to_state(x_target)
    
    # dist = distance_points(x_start, x_target)

    # return (x_target - x_start) / dist * max_dist + x_start

    ## Optimization routine (slow)
    x_start = tuple_to_state(x_start)
    x_target = tuple_to_state(x_target)
    min_dist = np.inf
    best_state = None

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

    return best_state

# RRT algorithm
def RRT(x_start:np.array, x_goal:np.array, max_iters:int = 1000)->tuple:
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
        x_new = connect(nearest_state, sample)
        if valid_state(x_new):
            # makin x_new a child of nearest_state
            tree[nearest_state].append(x_new) 
            # add a node to the tree
            tree[state_to_tuple(x_new)] = [] 
            if distance_points(x_new, x_goal) <= 0.5:
                return True, tree
    
    return False, tree

def visualize_without_obstacles(tree:dict)->None:
    """
    Visualize the tree of states without obstacles

    Args:
        tree: tree of states
    Returns: None
    """
    
    fig = plt.figure()
    # ax = plt.axes(projection='3d')
    ax = fig.add_subplot(111)

    for s in tree:
        ax.plot(s[0], s[1], '.', zorder=2)

        for c in tree[s]:
            ax.plot([s[0], c[0, 0]], [s[1], c[1, 0]], zorder=1)

    plt.ylim(-5., 5.)
    plt.xlim(-5., 5.)
    # ax.set_zlim(-5., 5.)
    plt.show()

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

def visualize_with_obstacles(tree:dict, obstacles:list)->None:
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

