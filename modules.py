import numpy as np
from modeling import simulate

r = .1
d = .25
rectangle_width = 4*d
rectangle_height = 2*d


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

def state_to_tuple(xx:np.ndarray)->tuple:
    """ 
    Convert a state vector to a tuple 

    Args:
        xx: state
    Returns:
        state in tuple form
    """
    x = [xx[i, 0] for i in range(len(xx))]
    x = tuple(x)
    
    return x

def tuple_to_state(t:tuple)->np.ndarray:
    """ Convert a tuple to a state
    
    Args:
        t: tuple
    Returns:
        state in np.array form
    """
    return np.array([[t[i] for i in range(len(t))]]).T

def nice_print_tree(tree:dict)->None:
    """
    Print the tree in a nice format

    Args:
        tree: tree of states
    Returns: None
    """
    for i, (key, value) in enumerate(tree.items()):
        print("-----------------------")
        print(f"Parent {i}:(", end="")
        for k in key:
            print(f"{k:.2f},", end=" ")
        print(")")
        print(f"Children: ")
        for child in value:
            print("(", end="")
            for j, item in enumerate(child):
                print(f"{item[0]:.2f},", end=" ")
            print(")")

def find_key_by_value(tree:dict, target_value:np.ndarray)->tuple:
    """

    Find the key of a value in a dictionary
    Args:
        tree: tree of states
        target_value: value to find
    Returns:
        key of the value
    """
    for key, value in tree.items():
        for v in value:
            if np.all(target_value == v):
                return key
    return None    
   
def best_path(tree:dict, start:tuple, target:tuple)->list:
    """

    Find the best path from the start to the target node

    Args:
        tree: tree of states
        start: start node
        target: target node
    Returns:
        best path from start to target node
    """
    path = []
    current = target
    while current != start:
        path.append(current)
        current = find_key_by_value(tree, tuple_to_state(current))
    path.append(start)
    path.reverse()

    return path

def optimal_control(x_start:tuple, x_target:tuple, max_iters:int = 1000, obstacles:list=[])->np.ndarray:
    """
    Connect the start state to the target state via an optimized path

    Args:
        x_start: start state
        x_target: target state
        max_iters: number of iterations
        obstacles: list of obstacles
    Returns:
        best_state: new state
        states: trajectory from start to target
    """
    # Optimization routine (slow)
    x_start = tuple_to_state(x_start)
    x_target = tuple_to_state(x_target)
    min_dist = np.inf

    def sample_control()->np.ndarray:
        """Sample a control input"""
        return np.random.uniform(-0.5, 0.5, (2, 1))
    
    for _ in range(max_iters):
        u = sample_control()
        states = simulate(x_start, u, 0.1, 4., "rk")
        for state in states:
            if collide_obstacles(state_to_tuple(state), obstacles):
                break
        dist = distance_points(state_to_tuple(states[-1])[1:], x_target[1:])
        if dist < min_dist:
            min_dist = dist
            best_state = states[-1]
    
    return best_state, states

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




    
