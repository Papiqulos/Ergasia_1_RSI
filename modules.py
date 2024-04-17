import numpy as np

def state_to_tuple(xx:np.array)->tuple:
    """ Convert a state to a tuple 
    Args:
        xx: state
    Returns:
        state in tuple form
    """
    x = xx.reshape((-1, 1))
    return (x[0, 0], x[1, 0])

def tuple_to_state(t:tuple)->np.array:
    """ Convert a tuple to a state
    Args:
        t: tuple
    Returns:
        state in np.array form
    """
    return np.array([[t[0], t[1]]]).T