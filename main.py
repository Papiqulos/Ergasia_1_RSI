import numpy as np
from modules import distance_points, state_to_tuple, tuple_to_state, nice_print_tree, best_path
from rrt import RRT, RRT_obstacles
from modeling import simulate
from visuals import *



def task1():

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

    # Simulate the robot
    states = simulate(x0, u, dt, T, method)
    # Visualize the states of the robot
    visualize_states(states)

def task2():
    # Example of a system with 2 state variables(x, y) no orientation

    # Target position
    x_target = np.array([[4., -2.]]).T 

    # Initial position
    x_init = np.array([[-2., 0.]]).T 
    init_tuple = state_to_tuple(x_init)

    # Optimization flag
    opt = False

    ## No Obstacles
    """Uncomment for the case without obstacles"""

    # valid, tree, _ = RRT(x_init, x_target, opt, 0.1, 1000)
    # print(f"Path found: {valid}\nNumber of nodes: {len(tree)}")

    # # Getting the final state and printing its distance from the target state
    # states_tuples = list(tree.keys())
    # end = states_tuples[-1]
    # print(f"End state: {end}")
    # print(f"distance from target: {distance_points(end, state_to_tuple(x_target))}\n")
    # # nice_print_tree(tree) # Uncomment to see the tree

    # # Getting the best path from the tree and visualizing it
    # path = best_path(tree, init_tuple, end)
    # # visualize_tree_without_obstacles(tree) # Uncomment to see the tree
    # visualize_best_path(path)

    ## Obstacles
    obstacles = [(0., 0., 1.), (0., -3., 1.), (2., -1., 1.)]  # (x,y,radius)
    valid1, tree1, _ = RRT_obstacles(x_init, x_target, opt, obstacles, 0.1, 1000)
    print(f"Path found: {valid1}\nNumber of nodes: {len(tree1)}")

    # Getting the final state and printing its distance from the target state
    states_tuples = list(tree1.keys())
    end = states_tuples[-1]
    print(f"End state: {end}")
    print(f"distance from target: {distance_points(end, state_to_tuple(x_target))}")
    # nice_print_tree(tree) # Uncomment to see the tree

    # Getting the best path from the tree and visualizing it
    path = best_path(tree1, init_tuple, end)
    # visualize_tree_with_obstacles(tree) # Uncomment to see the tree
    visualize_best_path(path, obstacles)

def task3():

    # Target position
    x_target = np.array([[-6., 2., 0.]]).T

    # Initial position
    x_init = np.array([[-2., -2., 0.]]).T 

    target_tuple = state_to_tuple(x_target)
    init_tuple = state_to_tuple(x_init)
    
    # Initial and target state conditions
    cond1 = distance_points(init_tuple[1:], target_tuple[1:]) >= 2.
    cond2 = (abs(target_tuple[0] - init_tuple[0])) >= np.pi / 2.



    ## No Obstacles
    """Uncomment for the case without obstacles""" 

    # Check if the initial and target states are valid
    # if cond1 and cond2:
    #     opt  = True
    #     valid, tree, trajectories = RRT(x_init, x_target, opt, 0.5, 100)
    #     tr = [item for sublist in trajectories for item in sublist] # Concatenating the trajectories
    #     print(f"Path found: {valid}\nNumber of nodes: {len(tree)}")

    #     # Getting the final state and printing its distance from the target state
    #     states_tuples = list(tree.keys())               # All the parents in the tree
    #     end = states_tuples[-1]                         # The last parent in the tree
    #     states = []
    #     # Convert the states from tuples to np arrays
    #     for state in states_tuples:
    #         states.append(tuple_to_state(state))
        
    #     print(f"End state: {end}")
    #     print(f"distance from target state(only x, y): {distance_points(end[1:], state_to_tuple(x_target)[1:])}")

    #     # Getting the best path from the tree and visualizing it
    #     path = best_path(tree, init_tuple, end)
    #     # path_xy = [p[1:] for p in path]              # Uncomment to see the path(only x, y)
    #     # visualize_best_path(path_xy)                 # Uncomment to see the path(only x, y)
    #     best_states = [tuple_to_state(p) for p in path]
    #     visualize_states(best_states)
    #     # visualize_states(tr)                         # Uncomment to see the trajectories
        
    # else:
    #     print("Invalid")


    # Obstacles
    obstacles = [(0., 0., 1.), (0., -3., 1.)]  # (x,y,radius)
    # Check if the initial and target states are valid
    if cond1 and cond2:
        opt  = True
        valid, tree, trajectories = RRT_obstacles(x_init, x_target, opt, obstacles, 0.5, 100)
        tr = [item for sublist in trajectories for item in sublist] # Concatenating the trajectories
        print(f"Path found: {valid}\nNumber of nodes: {len(tree)}")

        # Getting the final state and printing its distance from the target state
        states_tuples = list(tree.keys())                # All the parents in the tree
        end = states_tuples[-1]                          # The last parent in the tree
        states = []
        # Convert the states from tuples to np arrays
        for state in states_tuples:
            states.append(tuple_to_state(state))
        
        print(f"End state: {end}")
        print(f"distance from target state(only x, y): {distance_points(end[1:], state_to_tuple(x_target)[1:])}")

        # Getting the best path from the tree and visualizing it
        path = best_path(tree, init_tuple, end)
        # path_xy = [p[1:] for p in path]               # Uncomment to see the path(only x, y)
        # visualize_best_path(path_xy)                  # Uncomment to see the path(only x, y)
        best_states = [tuple_to_state(p) for p in path]
        visualize_states(best_states, obstacles) 
        # visualize_states(tr, obstacles)               # Uncomment to see the trajectories(some still collide with the obstacles)
        
    else:
        print("Invalid")
    

if __name__ == "__main__":
    
    """Uncomment the task you want to run"""
    # Model the robot and visualize for a given control input
    # task1()

    # RRT algorithm for a given initial and target state with and without obstacles
    # task2()

    # RRT algorithm for a given initial and target state with and without obstacles for the differential drive robot
    task3()




