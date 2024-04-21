import numpy as np
from modeling import simulate

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
    Convert a state to a tuple 

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




    
