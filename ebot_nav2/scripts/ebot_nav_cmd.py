#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from ebot_docking.srv import DockSw 

from rclpy.node import Node
import time
from payload_service.srv import PayloadSW
from geometry_msgs.msg import Twist
"""
Basic navigation demo to follow a given path after smoothing
"""

class Task2(Node):

    def __init__(self) -> None:
        super().__init__("task2")
        self.cli = self.create_client(DockSw, 'dock_control')
        # print("service callho gyi")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            print("service nhi mili")
        # self.req = self.cli.call_async(DockSw)
        self.req = DockSw.Request()
        self.payload_req_cli  = self.create_client(PayloadSW, '/payload_sw')
        self.vel = self.create_publisher(Twist, "/cmd_vel", 10)
       

        

    def call_dock_service(self,pose,distance):
        self.req.orientation = pose
        self.req.distance=distance
       
        return self.cli.call_async(self.req)
    
    def box_receive(self):
        # payload_req_cli  = self.create_client(PayloadSW, '/payload_sw')

        while not self.payload_req_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Payload service not available, waiting again...')

        req = PayloadSW.Request()
        req.receive = True     
        req.drop  = False       

        return self.payload_req_cli.call_async(req)

    def box_drop(self):
        # payload_req_cli  = self.create_client(PayloadSW, '/payload_sw')

        while not self.payload_req_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Payload service not available, waiting again...')

        req = PayloadSW.Request()
        req.receive = False    
        req.drop  = True       

        return self.payload_req_cli.call_async(req)    

    # def distance(self):
    #     vel = Twist()
    #     dist=5
    #     kp=2
    #     velocity=dist * kp
    #     vel.angular.z = velocity
    #     print("velocity is ",velocity)
    #     self.vel.publish(vel)
    #     print("align ho rha hai")





def main():
    rclpy.init()
    service=Task2()

    navigator = BasicNavigator()

    

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 1.8397628437589986
    initial_pose.pose.position.y = -9.049929750798595
    initial_pose.pose.orientation.z = 0.9999994711693747
    initial_pose.pose.orientation.w = 0.0007963
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # Go to our demos first goal pose
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = -0.12  #-4.4,  2.89
    goal_pose1.pose.position.y = -2.35
    goal_pose1.pose.orientation.w = 3.14

    # Get the path, smooth it
    path = navigator.getPath(initial_pose, goal_pose1)
    smoothed_path = navigator.smoothPath(path)

    # Follow path
    navigator.followPath(smoothed_path)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated distance remaining to goal position: '
                + '{0:.3f}'.format(feedback.distance_to_goal)
                + '\nCurrent speed of the robot: '
                + '{0:.3f}'.format(feedback.speed)
            )
    future = service.call_dock_service(3.14,0.75)
    rclpy.spin_until_future_complete(service, future)
    future = service.box_receive()
    rclpy.spin_until_future_complete(service, future)
    
    # Go to our demos first goal pose
    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x =  2.47
    goal_pose2.pose.position.y = 2.56
    goal_pose2.pose.orientation.w = 0.97

    # Get the path, smooth it
    path = navigator.getPath(goal_pose1, goal_pose2)
    smoothed_path = navigator.smoothPath(path)

    # Follow path
    navigator.followPath(smoothed_path)
    conveyor_1_pose=[ -4.4,  2.89, -1.57]
    conveyor_2_pose=[ 2.32,  2.55, -1.57]
    

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated distance remaining to goal position: '
                + '{0:.3f}'.format(feedback.distance_to_goal)
                + '\nCurrent speed of the robot: '
                + '{0:.3f}'.format(feedback.speed)
            )        
    future = service.call_dock_service(conveyor_1_pose[2],0.1)
    rclpy.spin_until_future_complete(service, future)

    future = service.box_drop()
    rclpy.spin_until_future_complete(service, future)


    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = -0.12  #-4.4,  2.89
    goal_pose1.pose.position.y = -2.35
    goal_pose1.pose.orientation.w = 3.1

    # Get the path, smooth it
    path = navigator.getPath(initial_pose, goal_pose1)
    smoothed_path = navigator.smoothPath(path)

    # Follow path
    navigator.followPath(smoothed_path)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated distance remaining to goal position: '
                + '{0:.3f}'.format(feedback.distance_to_goal)
                + '\nCurrent speed of the robot: '
                + '{0:.3f}'.format(feedback.speed)
            )
    future = service.call_dock_service(3.14,0.75)
    rclpy.spin_until_future_complete(service, future)
    
    future = service.box_receive()
    rclpy.spin_until_future_complete(service, future)

    # # Go to our demos first goal pose
    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = -4.7
    goal_pose3.pose.position.y = 2.64
    goal_pose3.pose.orientation.w = 2.78

    # Get the path, smooth it
    path = navigator.getPath(goal_pose2, goal_pose3)
    smoothed_path = navigator.smoothPath(path)

    # Follow path
    navigator.followPath(smoothed_path)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated distance remaining to goal position: '
                + '{0:.3f}'.format(feedback.distance_to_goal)
                + '\nCurrent speed of the robot: '
                + '{0:.3f}'.format(feedback.speed)
            )


    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    
    future = service.call_dock_service(conveyor_2_pose[2],0.1)
    rclpy.spin_until_future_complete(service, future)

    future = service.box_drop()   
    rclpy.spin_until_future_complete(service, future)

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()