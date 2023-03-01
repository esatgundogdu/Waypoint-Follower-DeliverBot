## EB
## waypoint_follower.py
## 
## BLG456E Assignment 2 Skeleton
## 
## Notes to consier: Few helper functions and code snippets are already given to you. Examine the code carefully beforehand.
##
## If you want to make use of the map, use occupancy_grid variable.
##
## 
## STUDENT_ID:<INSERT_HERE_WITHIN_ARROWS>
import rclpy
from rclpy.node import Node
import sys
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseArray, PoseStamped, Pose
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
import math
import numpy as np
"""
HELPER FUNCTIONS
"""
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        # DO NOT CHANGE HERE
        # DO NOT CHANGE HERE
        # DO NOT CHANGE HERE
        # DO NOT CHANGE HERE
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


def publish_path(path, publisher):
    """
    Publishes the path as a PoseArray message
    """
    poses = []
    if path is None:
        return
    for p in path:
        # convert p from occupancy grid coordinates to map coordinates
        # p = (p[0]*map_resolution + map_origin.position.x, p[1]*map_resolution + map_origin.position.y)
        pose = PoseStamped()
        pose.pose.position.x = float(p[0])
        pose.pose.position.y = float(p[1])
        poses.append(pose)
    path_msg = Path()
    path_msg.header.frame_id = "map"
    path_msg.poses = poses
    publisher.publish(path_msg)

# Astar algorithm used to generate path 
class Astar:
    def is_valid(pos, occupancy_grid, map_width):
        x, y = pos
        # check if the position is within the grid
        if x < 0 or x >= map_width or y < 0 or y >= len(occupancy_grid)/map_width:
            return False
        # check if the position is an obstacle
        if occupancy_grid[y*map_width + x] == -1:
            return False
        if occupancy_grid[y*map_width + x] != 100:
            return True
        return False

    def get_occupancy_cost(pos, occupancy_grid, map_width):
        x, y = pos
        return occupancy_grid[y*map_width + x]

    def astar(start, goal, h, neighbors, occupancy_grid, map_width):
        """Return a list of tuples as a path from the given start to the given end in the given occupancy grid"""
        # The set of nodes already evaluated
        closed_set = set()
        # The set of currently discovered nodes that are not evaluated yet.
        # Initially, only the start node is known.
        open_set = {start}
        # For each node, which node it can most efficiently be reached from.
        # If a node can be reached from many nodes, came_from will eventually contain the
        # most efficient previous step.
        came_from = {}
        # For each node, the cost of getting from the start node to that node.
        g_score = {start: 0}
        # For each node, the total cost of getting from the start node to the goal
        # by passing by that node. That value is partly known, partly heuristic.
        f_score = {start: h(start, goal)}
        while open_set:
            # current := the node in open_set having the lowest f_score[] value
            current = min(open_set, key=lambda o: f_score.get(o, sys.maxsize))
            if current == goal:
                return Astar.reconstruct_path(came_from, current)
            open_set.remove(current)
            closed_set.add(current)
            for neighbor in neighbors(current):
                if not Astar.is_valid(neighbor, occupancy_grid, map_width):
                    continue
                if neighbor in closed_set:
                    continue
                tentative_g_score = g_score[current] + 1
                if neighbor not in open_set:
                    open_set.add(neighbor)
                elif tentative_g_score >= g_score.get(neighbor, 0):
                    continue
                # This path to neighbor is the best until now. Record it!
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + h(neighbor, goal) + Astar.get_occupancy_cost(neighbor, occupancy_grid, map_width)
        return []

    def reconstruct_path(came_from, current):
        """Reconstruct a path from the came_from dictionary."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def manhattan_distance(a, b):
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return (dx + dy)

    def euclidean_distance(a, b):
        
            return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def neighbors4(pos):
        """Return the 4 neighboring positions."""
        x, y = pos
        return [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]

    def neighbors8(pos):
        """Return the 8 neighboring positions."""
        x, y = pos
        return [(x-1, y), (x+1, y), (x, y-1), (x, y+1),
                (x-1, y-1), (x+1, y-1), (x-1, y+1), (x+1, y+1)]

# Pure pursuit algorithm used to path tracking
class PurePursuit:
    def __init__(self):
        self.wheelbase = 0.3
        self.linear_vel_max = 0.7
        self.l_d_default = 0.3
        self.angular_vel_max = 1.5

        self.path = []
        self.path = []
        self.occupancy_grid = []
        self.map_width = 0
        self.map_origin = Pose()
        self.map_resolution = 0.0

        self.path_received = False
        self.map_received = False
        self.goal_reached = False

    def receive_path(self, path):
        self.path = path
        self.path_received = True

    def receive_map(self, occupancy_grid, map_width, map_origin, map_resolution):
        self.occupancy_grid = occupancy_grid
        self.map_width = map_width
        self.map_origin = map_origin
        self.map_resolution = map_resolution
        self.map_received = True

    def is_valid(self, p, pose_x, pose_y, pose_yaw):
        # get poses on occupancy grid
        x = p[0]
        y = p[1]

        # convert to occupancy grid coordinates
        x = int((x - self.map_origin.position.x)/self.map_resolution)
        y = int((y - self.map_origin.position.y)/self.map_resolution)
        robot_x = int((pose_x - self.map_origin.position.x)/self.map_resolution)
        robot_y = int((pose_y - self.map_origin.position.y)/self.map_resolution)

        # check for obstacles
        for i in range(abs(robot_x - x)):
            for j in range(abs(robot_y - y)):
                if self.occupancy_grid[i + j*self.map_width + min(robot_x, x) + self.map_width*min(robot_y, y)] != 0:
                    print("Not valid")
                    return False
        return True

    # Find the target point TP as the intersection of the desired path with a circle of radius lookahead_dist around the robot.
    def get_target_point(self, pose, lookahead_dist):
        # Find the closest point on the path to the robot.
        closest_idx = self.get_closest_point_idx(pose)

        # Find the point on the path that is lookahead_dist away from the robot.
        target_idx = self.get_target_point_idx(pose, closest_idx, lookahead_dist)

        # Return the target point.
        return self.path[target_idx]

    # Find the closest point on the path to the robot.
    def get_closest_point_idx(self, pose):
        closest_idx = 0
        closest_dist = float('inf')

        for i, point in enumerate(self.path):
            dist = self.get_dist(pose, point)

            if dist < closest_dist:
                closest_idx = i
                closest_dist = dist

        return closest_idx

    # Find the point on the path that is lookahead_dist away from the robot.
    def get_target_point_idx(self, pose, closest_idx, lookahead_dist):
        target_idx = closest_idx

        # find target point on path by checking lookahead distance
        while self.get_dist(pose, self.path[target_idx]) < lookahead_dist:
            target_idx += 1

            if target_idx >= len(self.path):
                break

        if target_idx >= len(self.path):
            target_idx = len(self.path) - 1

        # check if target point is the last point of the path
        if target_idx == len(self.path) - 1:
            self.goal_reached = True

        return target_idx

    # Find the distance between two points.
    def get_dist(self, point1, point2):
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

    # returns steering angle in radians
    def get_steering(self, goal, pose):
        lookahead_dist = self.l_d_default

        alpha = math.atan2(goal[1] - pose[1], goal[0] - pose[0]) - pose[2]

        steering = math.atan2(2.0*self.wheelbase*math.sin(alpha), lookahead_dist)

        return steering

    # returns angular velocity in radians per second
    def steering_to_angular_vel(self, steering, linear_vel):
        return linear_vel / self.wheelbase * math.tan(steering)

    def get_control(self):
        # wait until path and map are received
        if not self.path_received or not self.map_received:
            return 0.0, 0.0

        lookahead_dist = self.l_d_default
        
        pose = (robotX_tf, robotY_tf, robot_yaw)
        # get target point on path
        goal = self.get_target_point(pose, lookahead_dist)

        # check if goal reached
        if self.goal_reached and self.get_dist(pose, goal) < 0.2:
            return 0.0, 0.0

        # get steering angle
        steering = self.get_steering(goal, pose)

        linear_vel = self.linear_vel_max

        # get angular velocity from steering value
        angular_vel = self.steering_to_angular_vel(steering, linear_vel)

        # limit linear velocity based on angular velocity
        if abs(angular_vel) > 2.3:
            linear_vel = linear_vel * 0.1

        elif abs(angular_vel) > 1.5:
            linear_vel = linear_vel * 0.3

        elif abs(angular_vel) > 1.0:
            linear_vel = linear_vel * 0.4

        elif abs(angular_vel) > 0.5:
            linear_vel = linear_vel * 0.6

        elif abs(angular_vel) > 0.15:
            linear_vel = linear_vel * 0.7

        # limit angular velocity
        if angular_vel > self.angular_vel_max:
            angular_vel = self.angular_vel_max
        elif angular_vel < -self.angular_vel_max:
            angular_vel = -self.angular_vel_max
        

        return linear_vel, angular_vel

robot_yaw, robotX_tf, robotY_tf = 0, 0, 0
odom = Odometry()
class Navigator(Node):
    """
    Navigator node to make robot go from location A to B. 
    [IMPORTANT]
    IMPLEMENT YOUR CODES WITHIN THIS CLASS (You can define helper functions outside the class if you want)
    [IMPORTANT]
    """
    def __init__(self):
        super().__init__('waypoint_follower')
        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.sub_route1 = self.create_subscription(
            PoseArray,
            '/route1',
            self.set_route1,
            10,
        )# subscribe first route
        self.sub_route2 = self.create_subscription(
            PoseArray,
            '/route2',
            self.set_route2,
            10
        ) # subscribe second route
        self.subscription_waypoint = self.create_subscription(
            PoseStamped,
            '/waypoint',
            self.waypoint_callback,
            10) # subscribe next waypoint
        self.publish_twist = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.cli = self.create_client(GetMap, '/map_server/map')
        #/map_server/map
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetMap.Request()
        
        ### MAP ###
        self.occupancy_grid = [] # OCCUPANCY GRID MESSAGE VARIABLE. You can also use this knowledge
                                # to handle obstacle.
        self.map_origin = Pose() # origin as a Pose type.
        self.map_width = 0 # width
        self.map_height = 0 # height
        self.map_resolution = 0 # size of each grid cell
        ###     ###
        self.tf_buffer = Buffer() # for transformation
        self.tf_listener = TransformListener(self.tf_buffer, self) # for transformation
        self.goal_dist_thld = 0.2 # max acceptable distance between robot and the goal
        self.is_route1_set= 0 # 0 or 1, if 1 route1 is acquired by the topic
        self.is_route2_set = 0 # 0 or 1, if 1 route2 is acquired by the topic
        self.route1 = PoseArray() # route1
        self.route2 = PoseArray() # route2
        self.waypoint = PoseStamped() # next wayoint
        self.prev_distance = 0
        self.chatty_map = False # you may set chatty_map to true if you want to see map on the terminal

        self.tf_flag = False
        self.map_flag = False
        self.path_flag = False
        self.path = [] # path to the goal
        # path publisher
        self.path_pub = self.create_publisher(
            Path,
            '/path',
            10
        )
        self.pure_pursuit = PurePursuit() # pure pursuit object

    def send_request(self):
        self.future = self.cli.call_async(self.req)

    # get path of 2 routes
    def get_all_paths(self):
        for i in range(len(self.route1.poses)):
            goal_x = int((self.route1.poses[i].position.x - self.map_origin.position.x) / self.map_resolution)
            goal_y = int((self.route1.poses[i].position.y - self.map_origin.position.y) / self.map_resolution)
            # get x and y coords of robot on occupancy grid
            if i != 0:
                pose_x = int((self.route1.poses[i-1].position.x - self.map_origin.position.x) / self.map_resolution)
                pose_y = int((self.route1.poses[i-1].position.y - self.map_origin.position.y) / self.map_resolution)
            else:
                pose_x = int((robotX_tf - self.map_origin.position.x) / self.map_resolution)
                pose_y = int((robotY_tf - self.map_origin.position.y) / self.map_resolution)
            # get the path
            path = Astar.astar((pose_x, pose_y), (goal_x, goal_y), Astar.manhattan_distance, Astar.neighbors4, self.occupancy_grid, self.map_width)
            self.path += path
        
        for i in range(len(self.route2.poses)):
            goal_x = int((self.route2.poses[i].position.x - self.map_origin.position.x) / self.map_resolution)
            goal_y = int((self.route2.poses[i].position.y - self.map_origin.position.y) / self.map_resolution)
            # get x and y coords of robot on occupancy grid
            if i != 0:
                pose_x = int((self.route2.poses[i-1].position.x - self.map_origin.position.x) / self.map_resolution)
                pose_y = int((self.route2.poses[i-1].position.y - self.map_origin.position.y) / self.map_resolution)
            else:
                pose_x = int((self.route1.poses[-1].position.x - self.map_origin.position.x) / self.map_resolution)
                pose_y = int((self.route1.poses[-1].position.y - self.map_origin.position.y) / self.map_resolution)
            # get the path
            path = Astar.astar((pose_x, pose_y), (goal_x, goal_y), Astar.manhattan_distance, Astar.neighbors4, self.occupancy_grid, self.map_width)
            self.path += path
        
        # convert p from occupancy grid coordinates to map coordinates
        if len(self.path) == 0:
            self.get_logger().info("No path found")
            return

        for i in range(len(self.path)):
            self.path[i] = (self.path[i][0] * self.map_resolution + self.map_origin.position.x, self.path[i][1] * self.map_resolution + self.map_origin.position.y)
        # update pure_pursuit path
        self.pure_pursuit.receive_path(self.path)
        self.get_logger().info("Path found")

    def set_route1(self, msg):
        if (self.is_route1_set == 0):
            self.route1 = msg
            self.is_route1_set = 1
        else:
            pass

    def set_route2(self, msg):
        if (self.is_route2_set == 0):
            self.route2 = msg
            self.is_route2_set = 1
        else:
            pass
    
    def waypoint_callback(self,msg):
        # if msg differs from the previous one, update waypoint
        if (msg.pose.position.x != self.waypoint.pose.position.x or msg.pose.position.y != self.waypoint.pose.position.y):
            if self.tf_flag and self.map_flag:
                self.waypoint = msg
                # self.get_path(self.waypoint)
                self.get_logger().info("Waypoint received")

    def scan_callback(self, msg):
        pass
        
    # get map from map server and add costmap to it
    def map_response(self):
        self.occupancy_grid = np.array(self.occupancy_grid)

        # add costmap to the map
        # get indexes of obstacles
        obstacles = np.where(self.occupancy_grid == 100)
        offset_size = 8
        cost_value = 5
        for i in range(len(obstacles[0])):
            j = 1
            while j < offset_size:
                # add cost to left of the obstacle
                if i - j > 0 and self.occupancy_grid[obstacles[0][i]-j] != 100:
                    self.occupancy_grid[obstacles[0][i]-j] = (offset_size - j)**2 * cost_value
                # add cost to right of the obstacle
                if i + j < len(obstacles[0]) and self.occupancy_grid[obstacles[0][i]+j] != 100:
                    self.occupancy_grid[obstacles[0][i]+j] = (offset_size - j)**2 * cost_value
                # add cost to top of the obstacle
                if i - j*self.map_width > 0 and self.occupancy_grid[obstacles[0][i]-j*self.map_width] != 100:
                    self.occupancy_grid[obstacles[0][i]-j*self.map_width] = (offset_size - j)**2 * cost_value
                # add cost to bottom of the obstacle
                if i + j*self.map_width < len(obstacles[0]) and self.occupancy_grid[obstacles[0][i]+j*self.map_width] != 100:
                    self.occupancy_grid[obstacles[0][i]+j*self.map_width] = (offset_size - j)**2 * cost_value
                # add cost to top-left of the obstacle
                if i - j*self.map_width - j > 0 and self.occupancy_grid[obstacles[0][i]-j*self.map_width-j] != 100:
                    self.occupancy_grid[obstacles[0][i]-j*self.map_width-j] = (offset_size - j)**2 * cost_value
                # add cost to top-right of the obstacle
                if i - j*self.map_width + j > 0 and self.occupancy_grid[obstacles[0][i]-j*self.map_width+j] != 100:
                    self.occupancy_grid[obstacles[0][i]-j*self.map_width+j] = (offset_size - j)**2 * cost_value
                # add cost to bottom-left of the obstacle
                if i + j*self.map_width - j < len(obstacles[0]) and self.occupancy_grid[obstacles[0][i]+j*self.map_width-j] != 100:
                    self.occupancy_grid[obstacles[0][i]+j*self.map_width-j] = (offset_size - j)**2 * cost_value
                # add cost to bottom-right of the obstacle
                if i + j*self.map_width + j < len(obstacles[0]) and self.occupancy_grid[obstacles[0][i]+j*self.map_width+j] != 100:
                    self.occupancy_grid[obstacles[0][i]+j*self.map_width+j] = (offset_size - j)**2 * cost_value
                j += 1

        # send map to pure pursuit
        self.pure_pursuit.receive_map(self.occupancy_grid, self.map_width, self.map_origin, self.map_resolution)

    def odom_callback(self, msg):
        global robotX, robotY, robot_yaw, robotX_tf, robotY_tf, odom
        
        odom = msg
        robotX = msg.pose.pose.position.x
        robotY = msg.pose.pose.position.y
        
        to_frame_rel = "odom"
        from_frame_rel = "base_footprint"
        try:
            # grab the latest available transform from the odometry frame 
            # (robot's original location - usually the same as the map unless the odometry becomes inaccurate) to the robot's base.
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            self.tf_flag = True
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        # Convert the quaternion-based orientation of the latest message to Euler representation in order to get z axis rotation
        _,_,robot_orient_z = euler_from_quaternion(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w)
        robotX_tf = t.transform.translation.x
        robotY_tf = t.transform.translation.y
        robot_yaw = robot_orient_z # # only need the z axis, degree of orientation, between pi and -pi

        # general navigate function
        self.navigate()

        if (self.chatty_map):
            # you may set chatty_map to true if you want to see map on the terminal
            # map is only acquired for once and does not change since then.
            self.get_logger().info(str(self.occupancy_grid))
            self.get_logger().info("Length of the map array:" + str(len(self.occupancy_grid)))
            self.get_logger().info("Height:" + str(self.map_height) + " Width:"+ str(self.map_height))
            self.get_logger().info("Origin of the map (Cell 0,0):" + str(self.map_origin))
            self.get_logger().info("Resolution (Size of each grid cell):" + str(self.map_resolution))

            self.chatty_map = False # avoid repetitive printing.

    def navigate(self):
        # wait for tf
        if self.tf_flag == False:
            return
        # wait for map
        if self.map_flag == False:
            if self.future.done():
                self.map_flag = True
                self.map_response()
            return
        # if tf, map and routes are ready, calculate the path
        if not self.path_flag and self.is_route1_set == 1 and self.is_route2_set == 1:
            self.get_all_paths()
            self.path_flag = True
            return

        # publish path to /path topic for visualization
        publish_path(self.path, self.path_pub)

        velocity_vec = Twist()

        # Pure Pursuit Algorithm
        linear_vel, angular_vel = self.pure_pursuit.get_control()

        velocity_vec.linear.x = linear_vel
        velocity_vec.angular.z = angular_vel        

        self.publish_twist.publish(velocity_vec)
        
def main(args=None):
    # DO NOT CHANGE HERE
    # DO NOT CHANGE HERE
    # DO NOT CHANGE HERE
    # DO NOT CHANGE HERE
    rclpy.init(args=args)

    navigator_node = Navigator()
    navigator_node.send_request() # send request to map server
    get_response=False
    while rclpy.ok():
        rclpy.spin_once(navigator_node) 
        if (navigator_node.future.done() & (not get_response)):
            # if future job is accomplished (GetMap) and not accomplished before
            navigator_node.get_logger().info("map is acquired")
            try:
                response = navigator_node.future.result() # get map response
                get_response = True # raise the response flag
                navigator_node.occupancy_grid= response.map.data # get the occupancy grid array
                navigator_node.map_height= response.map.info.height # get the occupancy grid array
                navigator_node.map_width= response.map.info.width # get the occupancy grid array
                navigator_node.map_origin= response.map.info.origin # get the occupancy grid array
                navigator_node.map_resolution= response.map.info.resolution # get the occupancy grid array
                
            except Exception as e:
                navigator_node.get_logger().info(e) # raise an error if response could not be acquired.
                get_response = False # lower the flag

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    navigator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()