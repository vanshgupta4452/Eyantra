#!/usr/bin/env python3

## Overview

# ###
# This ROS2 script is designed to control a robot's docking behavior with a rack. 
# It utilizes odometry data, ultrasonic sensor readings, and provides docking control through a custom service. 
# The script handles both linear and angular motion to achieve docking alignment and execution.
# ###

# Import necessary ROS2 packages and message types
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion

from ebot_docking.srv import DockSw  # Import custom service message
import math, statistics
from payload_service.srv import PayloadSW

'''
- Go to receive position
- Call Receive service
- Go to conveyer 2
- Alignment and docking
- Call drop service
- Go to receive position
- Call Receive service
- Alignment and docking
- Call drop service

Alignment and docking:
- Read IMU data and ultrasonic data
- convert IMU quaternion to euler angle
- check difference of yaw of bot and rack = alpha
- use P controller to publish angular velocity
- check ultrasonic distance
- move back (publish twist message to move back)


drop_point: [0.43, 2.43] # [X, Y] # The payload service will drop the box at this position.

conveyor_1 pose: [ -4.4,  2.89, -1.57]
conveyor_2 pose: [ 2.32,  2.55, -1.57]


'''


# Define a class for your ROS2 node
class MyRobotDockingController(Node):

    def __init__(self):
        print("hello")
        # Initialize the ROS2 node with a unique name
        super().__init__('my_robot_docking_controller')
        
        # Create a callback group for managing callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Subscribe to odometry data for robot pose information
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)

        # Subscribe to ultrasonic sensor data for distance measurements
        self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
        # Add another one here
        self.ultrasonic_rr_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)

        # Create a ROS2 service for controlling docking behavior, can add another custom service message
        self.dock_control_srv = self.create_service(DockSw, 'dock_control', self.dock_control_callback, callback_group=self.callback_group)

        # Create a publisher for sending velocity commands to the robot
        #
        self.vel = self.create_publisher(Twist, "/cmd_vel", 10)

        self.robot_pose = [0.0, 0.0, 0.0]

        

        # Initialize all  flags and parameters here
        self.is_docking = False
        self.dock_aligned = False
        
        #         
        # 
        # 
        # 
        # 
        # 
        # self.conveyor_poses = conveyor_poses
        # Initialize a timer for the main control loop
        self.controller_timer = self.create_timer(0.1, self.controller_loop)

    # Callback function for odometry data
    def odometry_callback(self, msg):
        # Extract and update robot pose information from odometry message
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        quaternion_array = msg.pose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robot_pose[2] = yaw

    # Callback function for the left ultrasonic sensor
    def ultrasonic_rl_callback(self, msg):
        self.usrleft_value = msg.range

    # Callback function for the right ultrasonic sensor
    def ultrasonic_rr_callback(self,msg):
        self.usrright_value = msg.range

    # Utility function to normalize angles within the range of -π to π (OPTIONAL)
    def normalize_angle(self, angle):
        if angle > 180:
            angle -= 360 
        return angle

    # Main control loop for managing docking behavior


    def controller_loop(self):
        
        

        # The controller loop manages the robot's linear and angular motion 
        # control to achieve docking alignment and execution
        if self.is_docking:
            # ...
            # Implement control logic here for linear and angular motion
            # For example P-controller is enough, what is P-controller go check it out !
            # ...
            kp=0.6
        
            yaw_diff=self.conveyor_pose-self.robot_pose[2]
            if yaw_diff>=0.03  or yaw_diff<=-0.03:
                vel = Twist()
                
                
                velocity=yaw_diff * kp
                vel.angular.z = velocity
                print("velocity is ",velocity)
                self.vel.publish(vel)
                print("align ho rha hai")
            
            else:
                self.is_docking = True
                self.dock_aligned= True
                print("DONE ALIGNMENT")


            if self.dock_aligned:
                min_dist=self.distance
                vel = Twist()    
                current_distance = min(self.usrleft_value, self.usrright_value)
                if current_distance > min_dist:
                   vel.linear.x = -current_distance*kp 
                   self.vel.publish(vel)
                   print("distaince is ",current_distance)
                   print("moving towards dock")
                else:
                    
                    vel.linear.x = 0.0
                    self.is_docking = False
                    self.vel.publish(vel)
                    print("distaince is ",current_distance)
                    
                    print("DONE DISTANCE")
                    
       

    # Callback function for the DockControl service
    def dock_control_callback(self, request, response):
        # Extract desired docking parameters from the service request
        #
        #

        
        self.is_docking = True
        self.dock_aligned = False
        self.conveyor_pose = request.orientation
        self.distance=request.distance

        self.normalize_yaw_conveyor = self.normalize_angle(self.conveyor_pose)
       
        self.normalize_yaw_robot = self.normalize_angle(self.robot_pose[2])

        # Reset flags and start the docking process
        #
        #

        # Log a message indicating that docking has started
        self.get_logger().info("Docking started!")

        # Create a rate object to control the loop frequency
        rate = self.create_rate(2, self.get_clock())

        # Wait until the robot is aligned for docking
        while self.is_docking:
            self.normalize_yaw_conveyor = self.normalize_angle(self.conveyor_pose)
            self.normalize_yaw_robot = self.normalize_angle(self.robot_pose[2])
            self.is_docking = True
            self.get_logger().info("Waiting for alignment...")
            self.controller_loop()
            rate.sleep()

        # Set the service response indicating success
        response.success = True
        response.message = "Docking control initiated"
        return response
    

     # Extract desired docking parameters from the service request
        

# Main function to initialize the ROS2 node and spin the executor
def main(args=None):
    rclpy.init(args=args)
    # conveyor_1_pose=[ -4.4,  2.89, -1.57]
    # conveyor_2_pose=[ 2.32,  2.55, -1.57]
    # conveyor_pose=[conveyor_1_pose,conveyor_2_pose]
    my_robot_docking_controller = MyRobotDockingController()

    executor = MultiThreadedExecutor()
    executor.add_node(my_robot_docking_controller)

    executor.spin()

    my_robot_docking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
