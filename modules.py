import numpy as np

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
    """ Convert a state to a tuple 

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
        # print(current)
        path.append(current)
        current = find_key_by_value(tree, tuple_to_state(current))
    path.append(start)
    path.reverse()

    return path




    
