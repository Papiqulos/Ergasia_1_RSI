import numpy as np
from modules import distance_points, state_to_tuple
from rrt import nearest, sample_state, connect, valid_state

r = .1
d = .25
rectangle_width = 4*d
rectangle_height = 2*d

# RRT algorithm with obstacles
def RRT_obstacles(x_start:np.ndarray, x_goal:np.ndarray, opt:bool, obstacles:list, max_dist:float, max_iters:int = 1000)->tuple:
    """
    Rapidly-exploring Random Tree algorithm with obstacles

    Args:
        x_start: start state
        x_goal: goal state
        opt: optimization flag
        obstacles: list of obstacles
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
        x_new, trajectory = connect(nearest_state, sample, opt, obstacles)
        trajectories.append(trajectory)
        if valid_state(x_new, obstacles):
            # makin x_new a child of nearest_state
            tree[nearest_state].append(x_new) 
            # add a node to the tree
            tree[state_to_tuple(x_new)] = [] 
            if distance_points(x_new, x_goal) <= max_dist:
                return True, tree, trajectories
    
    return False, tree, trajectories

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

