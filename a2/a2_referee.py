import os
from re import X
import rclpy
from rclpy.node import Node
import sys
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseStamped
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
import math
from time import time

XY_CLOSENESS_THRESH=0.2
COSTHETA_CLOSENESS_THRESH=0.5-0.5*math.cos(math.pi/4)

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
  qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
  qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
  qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
 
  return [qx, qy, qz, qw]

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
        


class Referee(Node):

    def __init__(self):
        super().__init__('a2_referee')

        self.subscription_map = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.publish_twist = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.publish_route1 = self.create_publisher(
            PoseArray,
            '/route1',
            10
        )# publish first route
        self.publish_route2 = self.create_publisher(
            PoseArray,
            '/route2',
            10
        ) # publish second route
        self.publish_wp = self.create_publisher(
            PoseStamped,
            '/waypoint',
            10
        ) # publish next waypoint
        timer_period = 1  # seconds
        

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.move_counter = 0
        self.start = -1
        self.end = -1
        self.hasStarted = False
        self.waypoints_path = "waypoints"
        self.pose_array1, self.time_array1 = self.prepare_route(route=1) # return pose_array1 and time_array1 for route1
        self.pose_array2, self.time_array2  = self.prepare_route(route=2) # return pose_array1 and time_array1 for route2
        self.timer = self.create_timer(timer_period, self.timer_callback) # call every second
        self.waypoint_index = 0 # index to track waypoints
        self.cumulative_duration_error = 0.0
        self.hasFinished = False
        self.elapsed_time=0
    def map_callback(self, msg):
        pass


    def odom_callback(self, msg):
        global robotX
        global robotY
        robotX = msg.pose.pose.position.x
        robotY = msg.pose.pose.position.y
        to_frame_rel = "odom"
        from_frame_rel = "base_footprint"
    
        #velocity_vec = Twist() # to fill the velocity message 
        """
        try and except code block for transformation between odom and base_footprint to get
        the robot's location.
        """
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        _,_,robot_orient_z = euler_from_quaternion(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w)# get yaw from quaternion orientation of the robot
        robotX_tf = t.transform.translation.x # get robot's coordinates at x axis
        robotY_tf = t.transform.translation.y # get robot's coordinates at y axis
        robot_yaw = robot_orient_z # only need the z axis, degree of orientation, between pi and -pi
        #self.get_logger().info('X:'+str(robotX_tf),throttle_duration_sec=0.5) # once at a half of a second
        #self.get_logger().info('Y:'+str(robotY_tf),throttle_duration_sec=0.5) # once at a half of a second
        #self.get_logger().info('Yaw:'+str(robot_yaw),throttle_duration_sec=0.5) # once at a half of a second
        
        if (self.hasStarted & (not self.hasFinished)):
            # if hasStarted is true, print elapsed seconds for the robot
            self.get_logger().info('Elapsed secs:'+ str(time() - self.start),throttle_duration_sec=1)
        if ((round(msg.twist.twist.linear.x,2) > 0.0) | (round(msg.twist.twist.angular.z,2) > 0.0)):
            # if robot has moved, set hasStarted as true and make the clock running!
            if (self.move_counter==0):
                self.get_logger().info('Clock has started, RUN TURTLEBOT')
                self.start = time()
                self.hasStarted = True
                with open('wpReport.txt', 'r+') as f:
                    f.truncate(0)
                f.close()   
            self.move_counter+=1
        if (self.waypoint_index!=18):
            # get current point!
            this_point=self.pose_array1.poses[self.waypoint_index] if self.waypoint_index <=7 else self.pose_array2.poses[self.waypoint_index-8]
            #self.get_logger().info(str(type(this_point)))
            linear_error=math.sqrt((robotX_tf-this_point.position.x)**2+(robotY_tf-this_point.position.y)**2) # linear error between the 
            # robot's location and the next waypoint
            _,_,waypoint_theta = euler_from_quaternion(this_point.orientation.x,this_point.orientation.y, 
            this_point.orientation.z,this_point.orientation.w) # get yaw of the next waypoint
            angle_error=0.5-0.5*math.cos((robot_yaw-waypoint_theta)) # calculate angular error

            this_point_stamped = PoseStamped() # create an empty Pose with time stamp Header
            this_point_stamped.pose = this_point # assign the pose as the next pose
            this_point_timestamp = int(self.time_array1[self.waypoint_index]) if self.waypoint_index <=7 else int(self.time_array2[self.waypoint_index-8])
            # get expected timestamp for this point
            waypoint_header = Header() # define a header for PoseArray
            stamp_ns = 0 # zero nanoseconds
            stamp_time = Time()
            stamp_time.sec = this_point_timestamp
            stamp_time.nanosec = stamp_ns
  
            waypoint_header.frame_id =  "map" # focus frame: map
            waypoint_header.stamp = stamp_time # assign timestamp
            this_point_stamped.header = waypoint_header
            #this_point_stamped.header = self.pose_array1.header if self.waypoint_index <=8 else self.pose_array2.header
            self.publish_wp.publish(this_point_stamped) # publish i'th Pose/Waypoint

            self.end = time()
            self.elapsed_time = self.end - self.start

            duration_error = self.elapsed_time - this_point_timestamp

            if linear_error<=XY_CLOSENESS_THRESH:
                wp_indicator = "WP"+str(self.waypoint_index)
                # if robot comes closer to the waypoint, consider it as achied and control the yaw of the robot
                self.get_logger().info( "ACHIEVED WAYPOINT "+str(self.waypoint_index) + " with lateness of "+str(duration_error))
                self.get_logger().info("  (linear error "+str(linear_error)+" angle_error "+str(angle_error)+")")
                if (angle_error<=COSTHETA_CLOSENESS_THRESH):
                    # if yaw is OK, orientation is achieved.
                    self.get_logger().info("ACHIEVED ORIENTATION")
                    wp_indicator = wp_indicator + "+"
                
                self.cumulative_duration_error = self.cumulative_duration_error + duration_error
                with open('wpReport.txt', 'a') as f:
                    f.write(wp_indicator)
                    f.write(","+str(duration_error))
                    f.write("\n")
                f.close()
                self.waypoint_index+=1
            with open('timeReport.txt', 'w') as f:
                f.write("Total elapsed time:")
                f.write(str(self.elapsed_time))
                #f.write("\n")
            f.close()
        else:
            if (self.hasFinished==False):
                self.get_logger().info("Package is succesfully delivered")
                self.get_logger().info("With total lateness of " + str(self.cumulative_duration_error))
                self.get_logger().info("Total elapsed time:"+ str(self.elapsed_time))
                with open('timeReport.txt', 'w') as f:
                    f.write("Total elapsed time:")
                    f.write(str(self.elapsed_time))
                    f.write("\nPackage is succesfully delivered\n")
                    f.write("With total lateness of ")
                    f.write(str(self.cumulative_duration_error))
                    #f.write("\n")
                f.close()
            self.hasFinished = True

    def timer_callback(self):

        self.get_logger().info("Publishing: route", once=True)

        self.publish_route1.publish(self.pose_array1) # publish the route (set of waypoints)
        self.publish_route2.publish(self.pose_array2) # publish the route (set of waypoints)

    

    def prepare_route(self,route=1):
        """
        route = 1, change between 1 or 2 to get different routes
        """
        pose_array = PoseArray()
        pose_list = []
        timestamp_list = []
        wp1_path = os.path.join(self.waypoints_path,"waypoint{}.txt".format(route))
        wp_time = os.path.join(self.waypoints_path,"time{}.txt".format(route))
        with open(wp1_path) as f:
            waypoints_txt = f.readlines()
        with open(wp_time) as f:
            waypoints_time_txt = f.readlines()
        for idx,waypoint in enumerate(waypoints_txt):
            x,y,yaw = waypoint.split(' ',2)
            x = float(x)
            y = float(y)
            yaw = float(yaw)
            waypoint_pose = Pose() # create a Pose object
            waypoint_pose.position.x = x # assign coordinates of the waypoint to new object
            waypoint_pose.position.y = y # assign coordinates of the waypoint to new object
            waypoint_pose.position.z = 0.0 # assign coordinates of the waypoint to new object

            qx,qy,qz,qw = get_quaternion_from_euler(0.0,0.0,yaw) # convert yaw (euler-format) into quaternion format
            waypoint_pose.orientation.x = qx # assign yaw of the waypoint to new object
            waypoint_pose.orientation.y = qy # assign yaw of the waypoint to new object
            waypoint_pose.orientation.z = qz # assign yaw of the waypoint to new object
            waypoint_pose.orientation.w = qw # assign yaw of the waypoint to new object
            pose_list.append(waypoint_pose) # store waypoint pose as list
            timestamp_list.append(waypoints_time_txt[idx])

        waypoint_header = Header() # define a header for PoseArray
        stamp = self.get_clock().now() # get current timestamp of the referee node
        stamp = stamp.nanoseconds
        stamp_s = stamp // 10**9
        stamp_ns = stamp % 10**9
        stamp_time = Time()
        stamp_time.sec = stamp_s
        stamp_time.nanosec = stamp_ns
        self.get_logger().info(str(stamp))
        self.get_logger().info(str(type(stamp)))
        waypoint_header.frame_id =  "map" # focus frame: map
        waypoint_header.stamp = stamp_time # assign timestamp

        pose_array.poses = pose_list # set PoseArray's Pose[] list as our waypoint array
        pose_array.header = waypoint_header # set PoseArray's header as our waypoint header
        self.get_logger().info(str(pose_array))
        self.get_logger().info(str(pose_array.poses))
        self.get_logger().info(str(type(pose_array)))
        return pose_array,timestamp_list

def main(args=None):
    rclpy.init(args=args)

    ref = Referee()
    
    rclpy.spin(ref)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
   
    
    ref.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()