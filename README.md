# Motion-Planning
Implementation of search-based motion planning algorithms such as A* and different variations of A* like ARA*, RTAA*, MTA* for interception of a mobile target. Executed and optimized the performance of the algorithms on various known and unknown maps with different map sizes, robot/target positions and target movement strategies.

#   There are 3 different algorithms implemented, variants of A* algorithm:

    1)  vanilla A*
    2)  A* with node expansion
    3)  A* with node expansion and heuristic update

#   A class named 'environment()' is created in robotplanner.py to initiate the environment and store different parameters.
    This class takes an extra argument called 'algo' which decides the algorithm to use.
    
    It can take 3 different integers as input:

    1)  algo=0 for vanilla A*
    2)  algo=1 for A* w/ node expansion
    3)  algo=2 for A* w/ node expansion and heuristic update
    
    The default is set to 1 for A* w/ node expansion since it gives the fastest results in almost all maps, but it can be changed as needed.

#   In the main.py file, the class 'environment()' is initialized as 'env' inside the 'runtest()' function before the 'for loop'. 
    Select the algorithm in the environment class.
    Run main.py to get the results.

#    The function 'robotplanner()' is the main function in robotplanner.py that executes the algorithm and returns the next node for the robot.

    The function 'Astar()' is the function that implements the actual A* algortihm.
    It takes 2 parameters for tuning the search process.

    1)  eps: epsilon parameter to trust heuristics more
    2)  N: number of nodes to expand in the closed list

    The default is set to eps=1 and N=500 as it works best for most maps but can be tuned as needed for different maps for different speed.
    Taking eps=2, N=100 also present viable options.

#   Various plots are taken for the trajectory of the robot in the environment, by storing the coordinates over time.
    However, the plotting code has not been shown here for the sake of simplicity while running the code. 

