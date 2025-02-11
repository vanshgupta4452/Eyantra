#!/usr/bin/env python3
from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult
import rclpy
from copy import deepcopy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from payload_service.srv import PayloadSW
from ebot_docking.srv import DockSw 
from rclpy.node import Node

        

class Service(Node):
    def __init__(self,waypoints):
        super().__init__('minimal_client_async')
        self.timer = self.create_timer(0.02, self.nav)
      
        self.waypoints = waypoints
        self.routes_points = []
        self.routes_pose = PoseStamped() 
        self.nav = BasicNavigator()
        # self.nav.waitUntilNav2Active()
        # self.cli = self.create_client(DockSw, 'dock_control')
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        # self.req = DockSw.Request()

    def nav(self):     
        init_pose=PoseStamped()
        init_pose.header.frame_id="map"
        init_pose.header.stamp= self.nav.get_clock().now().to_msg()
        init_pose.pose.position.x=1.84
        init_pose.pose.position.y=-9.05
        init_pose.pose.orientation.z=0.1
        init_pose.pose.orientation.w=3.14   
        self.nav.setInitialPose(init_pose)

        for route in self.waypoints:
            self.routes_pose = PoseStamped()
            self.routes_pose.header.frame_id = 'map'
            self.routes_pose.header.stamp = self.nav.get_clock().now().to_msg()
            self.routes_pose.pose.position.x = route[0]
            self.routes_pose.pose.position.y = route[1]
            self.routes_pose.pose.orientation.z = 0.0
            self.routes_pose.pose.orientation.w = route[2]
            self.routes_points.append(deepcopy(self.routes_pose))
            
            


        print(f"Route point added: x={self.routes_pose.pose.position.x}, "
        f"y={self.routes_pose.pose.position.y}, "
        f"orientation.w={self.routes_pose.pose.orientation.w}") 


        self.nav.waitUntilNav2Active()
        self.nav.followWaypoints(self.routes_points)
        self.call_dock_service()


        i = 0
        while not self.nav.isTaskComplete():
            i += 1
            feedback = self.nav.getFeedback()
            if feedback and i % 5 == 0:
             print(
                'Executing current waypoint: '
                + str(feedback.current_waypoint + 1)
                + '/'
                + str(len(self.routes_points))
            )
        result = self.nav.getResult()
        # self.cli = self.create_client(DockSw, 'dock_control')
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        # self.req = DockSw.Request()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')

    def call_dock_service(self):
        self.cli = self.create_client(DockSw, 'dock_control')
        print("service callho gyi")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = self.cli.call_async(DockSw)

def main():
    rclpy.init()
   
    # ...


    #Routes to which we have to move
    routes=[
        [ -0.12, -2.35, 3.14 ],
        [ 1.86, 2.56, 0.97 ],
        [ -0.12, -2.35, 3.14 ],
        [ -3.84, 2.64, 2.78 ]
        ]
    node = Service(waypoints=routes)
    
    #making routes points
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
   