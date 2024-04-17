import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle 
import copy
import scipy

r = 0.1
d = .25

# x = [theta, x, y]^T
# u = [uL, uR]^T

# Erwthma 1.1
def diffkin(x:np.array, u:np.array)->np.array:
    """
    Differential kinematics model of the robot
    Args:
        x: state of the robot
        u: control input
    Returns:
        continuous kinematics of the robot
    """
    a = np.array([[-r / ( 2 * d ), r / ( 2 * d )], 
                  [( r / 2 ) * np.cos(x[0, 0]), ( r / 2 ) * np.cos(x[0, 0])], 
                  [( r / 2 ) * np.sin(x[0, 0]), ( r / 2 ) * np.sin(x[0, 0])]])
    
    return a @ u

# Erwthma 1.2
def visualize(states:list)->None:
    """
    Visualize the states of the robot
    Args:
        states: list of states of the robot
    Returns: None
    """
    if states:
        fig = plt.figure()
        ax = fig.add_subplot(111)
        for state in states[::2]:
            rect = Rectangle((state[1,0], state[2,0]), 4*d, 2*d, edgecolor = 'black', fill=False, angle=state[0, 0] * 180. / np.pi)
            # add rectangle to plot
            ax.add_patch(rect)

        plt.xlim(-2, 2)
        plt.ylim(-2, 2)
        plt.show()
    else:
        return
        
# Erwthma 1.3
def simulate(x0:np.array, u:np.array, dt:float, T:float, method=str)->list:
    """
    Simulate the robot with Euler or Runge-Kutta method
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
        if method == "euler":
            for k in range(K):
                x = states[k] + diffkin(states[k], u) * dt
                states.append(x)
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


def main():
    # Initial state of the robot
    x0 = np.array([[0., -1., 0.]]).T 
    # Control input
    u = np.array([[0.5, .2]]).T      
    # Time step
    dt = 0.1
    # Total time
    T = 4.
    states = simulate(x0, u, dt, T, "rk")
    visualize(states)
    
        

if __name__ == "__main__":
    main()




