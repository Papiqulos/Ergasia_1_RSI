import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import numpy as np

r = .1
d = .25

# Erwthma 1.2
def visualize_states(states:list)->None:
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

        plt.xlim(-20, 20)
        plt.ylim(-20, 20)
        plt.show()
    else:
        return

def visualize_tree(tree:dict, obstacles:list=[])->None:
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

    if obstacles:
        for o in obstacles:
            ax.add_patch(Circle([o[0], o[1]], radius=o[2], fill=False, zorder=3))

    plt.ylim(-5., 5.)
    plt.xlim(-5., 5.)
    plt.show()

def visualize_best_path(path:list, obstacles:list=[])->None:
    """
    Visualize the best path from the start to the target state

    Args:
        path: best path
    Returns: None
    """
    fig = plt.figure()
    ax = fig.add_subplot(111)

    for i in range(len(path) - 1):
        ax.plot(path[i][0], path[i][1], '.', zorder=2)
        ax.plot(path[i+1][0], path[i+1][1], '.', zorder=2)
        ax.plot([path[i][0], path[i + 1][0]], [path[i][1], path[i + 1][1]], zorder=1)
        
        if obstacles:
            for o in obstacles:
                ax.add_patch(Circle([o[0], o[1]], radius=o[2], fill=False, zorder=3))

    plt.ylim(-5., 5.)
    plt.xlim(-5., 5.)
    plt.show()