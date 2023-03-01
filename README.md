# Waypoint-Follower-DeliverBot
- This repository includes the solution for the "Waypoint Follower DeliverBot" problem which is an assignment given in Robotics course at Istanbul Technical University. Details about the assignment are given in file named "Waypoint_Follower_DeliverBot.pdf". The solution of the problem is in "a2/waypoint_follower.py".
- Running instructions are given in pdf file.

## Solution report
Two stages are required for the robot to follow the given waypoints without hitting obstacles.
1. Creating a path:
    - I used the A* algorithm to generate the path. Path generating steps are as follows:
        1. First of all, I added a cost map on the occupancy grid. I added exponentially decreasing costs to the grids around all obstacles in the occupancy grid.
        2. While creating the path, I added the cost in the occupancy grid as well as the g and heuristic values to the f value. Thus, the path passes as far as possible from the obstacles.
        3. I used the Manhattan Distance method as the heuristic function.
        4. I calculated separate paths for all waypoints in Route 1 and 2. Then, by combining these paths, I got a single path.
    - I forwarded the path I obtained to the next stage, the path tracking.
2. Path tracking:
    - I used the Pure Pursuit algorithm for path tracking. The stages are as follows
        1. First of all, I fixed the "lookahead distance" parameter of the algorithm.
        2. Then I chose a suitable target point on the given path using “lookahead distance” parameter.
        3. I determined the "steering angle" necessary for the robot to reach the target point by using the algorithm formulas. Then I converted this value to "angular velocity".
        4. Finally, I limited the "linear velocity" and "angular velocity" for a smooth tracking.
        5. I optimized some fixed parameters of the algorithm to be suitable for the robot.
    - Stages ii to iv are repeated every time odometry data arrives.