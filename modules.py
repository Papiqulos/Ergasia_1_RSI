import numpy as np

def state_to_tuple(xx:np.array)->tuple:
    """ Convert a state to a tuple 

    Args:
        xx: state
    Returns:
        state in tuple form
    """
    x = [xx[i, 0] for i in range(len(xx))]
    x = tuple(x)
    
    return x

def tuple_to_state(t:tuple)->np.array:
    """ Convert a tuple to a state
    
    Args:
        t: tuple
    Returns:
        state in np.array form
    """
    return np.array([[t[i] for i in range(len(t))]]).T