import numpy as np
from modules import *
from rrt import *
from modeling import *



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

    

    valid, tree = RRT(x_init, x_target, 100)
    print(valid, len(tree))
    states_tuples = list(tree.keys())
    end = states_tuples[-1]
    print(f"End state: {end}")
    print(f"distance from target: {distance_points(end, state_to_tuple(x_target))}")
    visualize_without_obstacles(tree)

    
    # for i, (key, value) in enumerate(tree.items()):
    #     print(f"Parent: {key}")
    #     for v in value:
    #         print(f"--children : theta: {v[0]}, x: {v[1]}, y: {v[2]}")
    
    # if valid:
    #     visualize_without_obstacles(tree)

    # Obstacles
    obstacles = [(0., 0., 1.), (0., -3., 1.)]  # (x,y,radius)

    # valid1, tree1 = RRT_obstacles(x_init, x_target, obstacles, 500)
    # print(valid1, len(tree1))
    # visualize_with_obstacles(tree1, obstacles)

    

def erwthma3():

    # Target position
    x_target = np.array([[-6., 10., 14.5]]).T 

    # Initial position
    x_init = np.array([[-2., 0., 1.]]).T 

    target_tuple = state_to_tuple(x_target)
    init_tuple = state_to_tuple(x_init)
    
    cond1 = distance_points(init_tuple[1:], target_tuple[1:]) >= 2.
    cond2 = (abs(target_tuple[0] - init_tuple[0])) >= np.pi / 2.
    

    if cond1 and cond2:
        print("Valid")

        valid, tree = RRT(x_init, x_target, 100)
        print(valid, len(tree))

        states_tuples = list(tree.keys())
        end = states_tuples[-1]
        states = []
        for state in states_tuples:
            states.append(tuple_to_state(state))
        
        print(f"End state: {end}")
        print(f"distance from target state(includes theta): {distance_points(end, state_to_tuple(x_target))}")
        visualize_states(states)
    else:
        print("Invalid")


    # Obstacles
    obstacles = [(0., 0., 1.), (0., -3., 1.)]  # (x,y,radius) 
    
    



if __name__ == "__main__":

    # erwthma1()

    # erwthma2()

    erwthma3()




