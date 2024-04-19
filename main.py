import numpy as np
from modules import *
from rrt import RRT, visualize2D_without_obstacles
from rrt_with_obstacles import RRT_obstacles, visualize2D_with_sobstacles
from modeling import simulate, visualize_states



def erwthma1():

    # Initial state of the robot
    x0 = np.array([[0., -1., 0.]]).T 
    # Control input
    u = np.array([[.5, .4]]).T      
    # Time step
    dt = 0.5
    # Total time
    T = 20.
    # Integration method
    method = "rk"

    states = simulate(x0, u, dt, T, method)
    visualize_states(states)

def erwthma2():
    # Example of a system with 2 state variables

    # Target position
    x_target = np.array([[-3., 0.]]).T 

    # Initial position
    x_init = np.array([[-2., 0.]]).T 

    
    # No Obstacles
    opt = False
    valid, tree = RRT(x_init, x_target, opt, 100)
    print(valid, len(tree))
    states_tuples = list(tree.keys())
    end = states_tuples[-1]
    print(f"End state: {end}")
    print(f"distance from target: {distance_points(end, state_to_tuple(x_target))}")
    visualize2D_without_obstacles(tree)

    # Obstacles
    # obstacles = [(0., 0., 1.), (0., -3., 1.)]  # (x,y,radius)
    # valid1, tree1 = RRT_obstacles(x_init, x_target, obstacles, 500)
    # print(valid1, len(tree1))
    # states_tuples = list(tree1.keys())
    # end = states_tuples[-1]
    # print(f"End state: {end}")
    # print(f"distance from target: {distance_points(end, state_to_tuple(x_target))}")
    # visualize_with_obstacles(tree1, obstacles)

def erwthma3():

    # Target position
    x_target = np.array([[-6., 5., 2.5]]).T 

    # Initial position
    x_init = np.array([[-2., 0., 1.]]).T 

    target_tuple = state_to_tuple(x_target)
    init_tuple = state_to_tuple(x_init)
    
    # Initial and target state conditions
    cond1 = distance_points(init_tuple[1:], target_tuple[1:]) >= 2.
    cond2 = (abs(target_tuple[0] - init_tuple[0])) >= np.pi / 2.
    
    # Check if the initial and target states are valid
    if cond1 and cond2:
        print("Valid")
        opt  = True
        valid, tree = RRT(x_init, x_target, opt, 100)
        print(valid, len(tree))
        # Check if a path was found
        if valid :
            print("Path found")
        else:
            print("Path not found")

        # Visualize the states (parent nodes of tree)
        states_tuples = list(tree.keys())
        end = states_tuples[-1]
        states = []
        for state in states_tuples:
            states.append(tuple_to_state(state))
        
        # Print the end state and the distance from the target state
        print(f"End state: {end}")
        print(f"distance from target state(only x, y): {distance_points(end[1:], state_to_tuple(x_target)[1:])}")
        visualize_states(states)
    else:
        print("Invalid")


    # Obstacles
    # obstacles = [(0., 0., 1.), (0., -3., 1.)]  # (x,y,radius) 
    
    



if __name__ == "__main__":

    # erwthma1()

    erwthma2()

    # erwthma3()




