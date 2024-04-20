import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

r = .1
d = .25
rectangle_width = 4*d
rectangle_height = 2*d

# x = [theta, x, y]^T
# u = [uL, uR]^T

# Erwthma 1.1
def diffkin(x:np.ndarray, u:np.ndarray)->np.ndarray:
    """
    Differential kinematics model of the robot

    Args:
        x: state of the robot
        u: control input
    Returns:
        continuous kinematics of the robot
    """
    a = np.array([[-r / ( 2. * d ), r / ( 2. * d )], 
                  [( r / 2. ) * np.cos(x[0, 0]), ( r / 2. ) * np.cos(x[0, 0])], 
                  [( r / 2. ) * np.sin(x[0, 0]), ( r / 2. ) * np.sin(x[0, 0])]])
    
    return a @ u

# Erwthma 1.3
def simulate(x0:np.ndarray, u:np.ndarray, dt:float, T:float, method:str = "rk")->list:
    """
    Simulate the robot with Euler or Runge-Kutta 4th order Integration
    
    Args:
        x0: initial state of the robot
        u: control input
        dt: time step
        T: total time
    Returns:
        list of states of the robot
    """

    # assert np.any(u < 0.5)
    # Check if control input is valid
    if np.all(u <= 0.5):

        K = int(T/dt) + 1
        states = [x0]

        # Euler Integration
        if method == "euler":
            for k in range(K):
                x = states[k] + diffkin(states[k], u) * dt
                states.append(x)
        # Runge-Kutta 4th order Integration
        elif method == "rk":
            for k in range(K):
                x = states[k]
                k1 = diffkin(x, u)
                k2 = diffkin(x + k1 * dt / 2, u)
                k3 = diffkin(x + k2 * dt / 2, u)
                k4 = diffkin(x + k3 * dt, u)
                x = x + (k1 + 2 * k2 + 2 * k3 + k4) * dt / 6
                states.append(x)

        return states
    else:
        print("Invalid control input, must be less than 0.5")
        return 