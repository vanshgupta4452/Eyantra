#!/usr/bin/env python3
"""
Script to control a UR5 robotic arm using ROS2. The arm moves through a sequence of waypoints 
in the order: P1 → D → P2 → D → P3 → D, attaching and detaching objects at each step.
"""

import time
from os import path
from threading import Thread
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from linkattacher_msgs.srv import AttachLink, DetachLink

# Define target positions and orientations
WAYPOINTS = [
    {"position": [0.20, -0.47, 0.65], "quat": [0.7077, 0.0038, 0.0007, 0.7065], "box_id":[1]},
    {"position": [0.75, 0.49, -0.05], "quat": [0.7103, 0.7038, -0.0021, -0.0081], "box_id":[3]},
    {"position": [0.75, -0.23, -0.05], "quat": [0.7104, 0.7037, 0.0127, 0.0069], "box_id":[49]}
]

DETACH_POSITION = {"position": [-0.69, 0.10, 0.44], "quat": [-0.6837, 0.7265, 0.0528, -0.0440]}

# Adding meshes for obstacle avoidance
RACK_MESH = path.join(
	path.dirname(path.realpath(__file__)), "assets", "rack.stl"
)
rackpos=[0.25, -0.645002, -0.58]
rackqua=[ 0.0, 0.0, 0.7068252, 0.7073883 ]

BOX_MESH=path.join(
	path.dirname(path.realpath(__file__)), "assets", "box.stl"
)

initial_pos=[-0.0349, -1.8850, 1.6232, -1.5533, -1.5533, 3.1241]
box1_pos=[0.20, -0.65, 0.60]
box1_qua=[0.000242,0.000000,-0.707,0.707]
box2_pos=[0.70, -0.23, -0.20]
box2_qua=[0.51183, 0.51183, 0.48788, 0.48788]
box3_pos=[0.70, 0.49, -0.20]
box3_qua=[0.50895, 0.51479, 0.49212, 0.48351]
rack_base_pos=[0.0,0.0,0.0]
rack_base_qua=[0.51183, 0.51183, 0.48788, 0.48788]
boxpos=[box1_pos,box2_pos,box3_pos]
boxqua=[box1_qua,box2_qua,box3_qua]
        
filepath1 = RACK_MESH
filepath2 = BOX_MESH
filepath3=RACK_MESH

class GripperControl(Node):
    def __init__(self):
        super().__init__('gripper_control')
        
        # Initialize clients for attaching and detaching services
        self.gripper_attach_client = self.create_client(AttachLink, '/GripperMagnetON')
        self.gripper_detach_client = self.create_client(DetachLink, '/GripperMagnetOFF')
        self.callback_group = ReentrantCallbackGroup()

        # Initialize MoveIt2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=ur5.joint_names(),
            base_link_name=ur5.base_link_name(),
            end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM,
            callback_group=self.callback_group
        )
        
        self.declare_parameter(
        "initial_pos", initial_pos)

        self.declare_parameter(
        "drop pos", [0.15707963267948966, -1.7278759594743862, -1.2566370614359173, -0.06981317007977318, 1.4311699866353502, 0.03490658503988659]
)

        self.joint_positions0 = self.get_parameter("initial_pos").get_parameter_value().double_array_value
        self.joint_positions1 = self.get_parameter("drop_pos").get_parameter_value().double_array_value

        # Spin the node in background thread(s)
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(self)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()

    def move_to_pose(self, position, quat, cartesian=True):        
        self.moveit2.move_to_pose(position=position, quat_xyzw=quat, cartesian=cartesian)
        self.moveit2.wait_until_executed()

    def add_coll_meshes(self):
        self.moveit2.add_collision_mesh(
                filepath=filepath1, id="rack2", position=rackpos, quat_xyzw=rackqua, frame_id=ur5.base_link_name()
            )
        for j in range(3):
            self.moveit2.add_collision_mesh(
                    filepath=filepath2, id="box"[j], position=boxpos[j], quat_xyzw=boxqua[j], frame_id=ur5.base_link_name()
                )
        self.get_logger().info("Collision meshes added successfully")

    def attach_gripper(self, box_id):
        req = AttachLink.Request()
        req.model1_name = "box" + str(box_id[0])
        req.link1_name = 'link'
        req.model2_name = 'ur5'
        req.link2_name = 'wrist_3_link'
        self.get_logger().info(f"{req.model1_name}")
        self.get_logger().info("Attempting to call the Attach service for the box")
        self._call_service(self.gripper_attach_client, req)
        self.get_logger().info("Service call completed")

    def detach_gripper(self, box_id):
        req = DetachLink.Request()
        req.model1_name = "box" + str(box_id[0])
        req.link1_name = 'link'
        req.model2_name = 'ur5'
        req.link2_name = 'wrist_3_link'
        self.get_logger().info("here we detached the gripper ")
        self._call_service(self.gripper_detach_client, req)

    def _call_service(self, client, req):
        self.get_logger().info("Service call completed again")

        while not client.wait_for_service(timeout_sec=4.0):
            self.get_logger().info('EEF service not available, waiting again...')
            self.get_logger().info('action')
        client.call_async(req)
        
    def execute_movement_sequence(self):
        for waypoint in WAYPOINTS:
            self.get_logger().info("moving to waypoint")
            # Move to pickup point and attach the object
            self.moveit2.move_to_configuration(self.joint_positions0)
            self.move_to_pose(waypoint["position"], waypoint["quat"])
            self.get_logger().info("From main: Attempting to call the Attach service for the box")
            self.attach_gripper(waypoint["box_id"])
            
            # Move to detach point and release the object
            # self.move_to_pose(DETACH_POSITION["position"], DETACH_POSITION["quat"])
            self.moveit2.move_to_configuration(self.joint_positions1)
            self.detach_gripper(waypoint["box_id"])

def main():
    rclpy.init()
    grip_control = GripperControl()
    grip_control.add_coll_meshes()
    grip_control.execute_movement_sequence()
    rclpy.shutdown()

if __name__ == "__main__":
    main()