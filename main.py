import numpy as np
from modules import *
from rrt import RRT
from rrt_with_obstacles import RRT_obstacles
from modeling import simulate
from visuals import *



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
    # Example of a system with 2 state variables(x, y) no orientation

    # Target position
    x_target = np.array([[4., -2.]]).T 

    # Initial position
    x_init = np.array([[-2., 0.]]).T 
    init_tuple = state_to_tuple(x_init)

    
    ## No Obstacles
    opt = False
    valid, tree = RRT(x_init, x_target, opt, 1000)
    print(f"Path found: {valid}\nNumber of nodes: {len(tree)}")

    # Getting the final state and printing its distance from the target state
    states_tuples = list(tree.keys())
    end = states_tuples[-1]
    print(f"End state: {end}")
    print(f"distance from target: {distance_points(end, state_to_tuple(x_target))}\n")
    # nice_print_tree(tree)

    # Getting the best path from the tree and visualizing it
    path = best_path(tree, init_tuple, end)
    # visualize_tree_without_obstacles(tree)
    visualize_best_path(path)

    ## Obstacles
    # obstacles = [(0., 0., 1.), (0., -3., 1.), (2., -1., 1.)]  # (x,y,radius)
    # valid1, tree1 = RRT_obstacles(x_init, x_target, obstacles, 1000)
    # print(f"Path found: {valid1}\nNumber of nodes: {len(tree1)}")

    # Getting the final state and printing its distance from the target state
    # states_tuples = list(tree1.keys())
    # end = states_tuples[-1]
    # print(f"End state: {end}")
    # print(f"distance from target: {distance_points(end, state_to_tuple(x_target))}")
    # # nice_print_tree(tree)

    # Getting the best path from the tree and visualizing it
    # path = best_path(tree1, state_to_tuple(x_init), end)
    # # visualize_tree_with_obstacles(tree)
    # visualize_best_path(path, obstacles)

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
        opt  = True
        valid, tree = RRT(x_init, x_target, opt, 1000)
        print(f"Path found: {valid}\nNumber of nodes: {len(tree)}")

        # Visualize the states
        states_tuples = list(tree.keys())
        end = states_tuples[-1]
        states = []
        for state in states_tuples:
            states.append(tuple_to_state(state))

        path = best_path(tree, init_tuple, end)
        best_states = [tuple_to_state(p) for p in path]
        # visualize_states(best_states)
        
        # Print the end state and the distance from the target state
        print(f"End state: {end}")
        print(f"distance from target state(only x, y): {distance_points(end[1:], state_to_tuple(x_target)[1:])}")
        
    else:
        print("Invalid")


    # Obstacles
    # obstacles = [(0., 0., 1.), (0., -3., 1.)]  # (x,y,radius) 
    
    



if __name__ == "__main__":

    # erwthma1()

    erwthma2()

    # erwthma3()




