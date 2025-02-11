#!/usr/bin/env python3
from math import cos, sin
import math, time
from copy import deepcopy
import rclpy
import tf2_ros
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from pymoveit2.robots import ur5
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from pymoveit2 import MoveIt2Servo

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from linkattacher_msgs.srv import AttachLink, DetachLink
from os import path
from threading import Thread

from pymoveit2 import MoveIt2

'''
list of boxes = [1, 3, 49]
drop position = xyz     "joint_positions1",
                           [0.0, -1.972219686249898, -1.0471975802753372, -3.141592653589793, -1.5882534483405718, 3.141592653589793],

iterate over every box id from list of boxes
    - call the servo function with parameter box id 

Servo Callback Function Steps:
- Takes box id
- Lookup transform
- Finds distance and velocity
- Publish message
- wait until distance is minimum in x,y,z
- gripper function()
- joint state function()
- gripper function()
- wait until execution
- self.i += 1 if not reached to end of the list yet
- kill the node

Node:
    list of boxes is already there
    i = 0
    callback function keeps running



P1:   [ 0.20, -0.47, 0.65 ]

P2:   [ 0.75,0.49,-0.05 ]

P3:   [ 0.75,-0.23,-0.05 ]


points = []

'''

RACK_MESH = path.join(
	path.dirname(path.realpath(__file__)), "assets", "rack.stl"
)
# ARM_BASE_MESH = path.join(
# 	path.dirname(path.realpath(__file__)), "assets", "arm_base.dae"
# )

BOX_MESH=path.join(
	path.dirname(path.realpath(__file__)), "assets", "box.stl"
)

class Task1C(Node):

    def __init__(self, box_ids):
        super().__init__('sample_tf2_frame_listener')
        print("---------------NODE CREATED-----------------")
        self.box_ids = box_ids
        self.i = 0
        self.timer = self.create_timer(0.02, self.move)
        self.twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.gripper_control = self.create_client(AttachLink, '/GripperMagnetON')
        self.gripper_control_off = self.create_client(DetachLink, '/GripperMagnetOFF')
        self.callback_group = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(
                node=self,
                joint_names=ur5.joint_names(),
                base_link_name=ur5.base_link_name(),
                end_effector_name=ur5.end_effector_name(),
                group_name=ur5.MOVE_GROUP_ARM,
                callback_group=self.callback_group,)
        self.declare_parameter(
            "joint_positions_drop",
            [-1.5707963267948966, -2.3876104167282426, 2.4085543677521746, -3.1590459461097367, -1.5707963267948966, 3.141592653589793],
       )
        
        self.declare_parameter(
        "joint_positions_initial",
        [-0.0349, -1.8850, 1.6232, -1.5533, -1.5533, 3.1241],
        
        ) 

        self.declare_parameter(
        "joint_positions1",
        [0.0, -1.972219686249898, -1.0471975802753372, -3.141592653589793, -1.5882534483405718, 3.141592653589793]
,
        
        )
        self.tf_buffer = Buffer()                                                                       # initializing transform buffer object
        self.tf_listener = TransformListener(self.tf_buffer, self) 
  
        

    def move(self):
            points=[ [ 0.20, -0.47, 0.65 ],[ 0.75,0.49,-0.05 ],[ 0.75,-0.23,-0.05 ]]              
            print("---------------MOVE FUNCTION CALLED------------")
            from_frame_rel = 'obj_1'                                                                        # frame from which transfrom has been sent
            to_frame_rel = 'base_link'                                                                      # frame to which transfrom has been sent
            if self.i != 0 :
                print("pos pe ja rha hu")
                drop_pos = self.get_parameter("joint_positions1").get_parameter_value().double_array_value
                self.moveit2.move_to_configuration(drop_pos)
                print("self.i===",self.i)
                self.moveit2.wait_until_executed()
                
                self.detach_gripper(self.box_ids[(self.i)-1])

                if self.i==3:
                    rclpy.shutdown()
                    exit(0)

                           
            
            
            if self.i==0:
                joint_positions = (
                    self.get_parameter("joint_positions_drop").get_parameter_value().double_array_value
                )
            else:
                joint_positions = (
                    self.get_parameter("joint_positions_initial").get_parameter_value().double_array_value
                )

            self.moveit2.move_to_configuration(joint_positions)
            self.moveit2.wait_until_executed()

            print("JOint pose reached")
            
            while True :

                try:
                    # box = self.tf_buffer.lookup_transform( to_frame_rel, f"obj_{self.box_ids[self.i]}", rclpy.time.Time())       # look up for the transformation between 'obj_1' and 'base_link' frames
                    tool0 = self.tf_buffer.lookup_transform('base_link', "tool0", rclpy.time.Time())
                    self.get_logger().info(f'Successfully received data!')
                    # box_orientation=[box.transform.rotation.x,box.transform.rotation.y,box.transform.rotation.z,box.transform.rotation.w]
                    # tool_0_orientation=[tool0.transform.rotation.x,tool0.transform.rotation.y,tool0.transform.rotation.z,tool0.transform.rotation.w]
                    
                    print("**********************************")
                    print("**********TRANSFORMS**************")
                    self.get_logger().info(f'Translation X:  {tool0.transform.translation.x}')
                    self.get_logger().info(f'Translation Y:  {tool0.transform.translation.y}')
                    self.get_logger().info(f'Translation Z:  {tool0.transform.translation.z}')
                    self.get_logger().info(f'Rotation X:  {tool0.transform.rotation.x}')                                # NOTE: rotations are in quaternions
                    self.get_logger().info(f'Rotation Y:  {tool0.transform.rotation.y}')
                    self.get_logger().info(f'Rotation Z:  {tool0.transform.rotation.z}')
                    self.get_logger().info(f'Rotation W:  {tool0.transform.rotation.w}')
                    print("**********************************")

                    dist_x=(points[self.i][0]) - (tool0.transform.translation.x)
                    dist_y=(points[self.i][1]) - (tool0.transform.translation.y)
                    dist_z=(points[self.i][2]) - (tool0.transform.translation.z)      
                    # rot_x=(box_orientation[0])-(tool_0_orientation[0])
                    # rot_y=(box_orientation[1])-(tool_0_orientation[1])
                    # rot_z=(box_orientation[2])-(tool_0_orientation[2])  
                    print("dist_X>...............",dist_x)
                    print("dist_X>...............",dist_y)
                    print("dist_X>...............",dist_z)                
                    __twist_msg = TwistStamped()
                    __twist_msg.header.stamp = self.get_clock().now().to_msg()
                    __twist_msg.header.frame_id = ur5.base_link_name()
                    __twist_msg.twist.linear.x = dist_x*10
                    __twist_msg.twist.linear.y = dist_y*10
                    __twist_msg.twist.linear.z = dist_z*10
                    # __twist_msg.twist.angular.x = rot_x/2
                    # __twist_msg.twist.angular.y = rot_y/2
                    # __twist_msg.twist.angular.z = rot_z/2
                    
                    self.twist_pub.publish(__twist_msg)
                    

                    print("publish ho rha hai")
                    print(__twist_msg)
                except TransformException as e:
                    self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {e}')
                    return

                if not (abs(dist_x) > 0.04 or  abs(dist_y) > 0.04 or abs(dist_z) > 0.04):
                    break

            print("WENT TO THE BOX POSITION")
            print("NOW CALLING PICK AND DROPPING FUNCTIONS")

            # ----- ATTACH GRIPPER FUNCTION ------
            self.attach_gripper(self.box_ids[self.i])

            # ----- JOINT FUNCTION CALL ------
            if self.i==0:
                joint_positions = (
                    self.get_parameter("joint_positions_drop").get_parameter_value().double_array_value
                )
            else:
                joint_positions = (
                    self.get_parameter("joint_positions_initial").get_parameter_value().double_array_value
                )


            # drop_pos = self.get_parameter("joint_positions1").get_parameter_value().double_array_value
            # self.moveit2.move_to_configuration(drop_pos)
            print("neeche walla")
            time.sleep(0.15)
            # self.moveit2.wait_until_executed()
            print("execute ho gya")




            # ----- DETACH FUNCTION CALL -------
           
            
            self.i += 1
            # if self.i == len(self.box_ids):
            #     rclpy.shutdown()
            #     exit(0)

    # function to detach
    def detach_gripper(self, box):
            

        while not self.gripper_control_off.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EEF service not available, waiting again...')

        print(" IN THE MIDDLE OF DETACH GRIPPER")    

        req = DetachLink.Request()
        req.model1_name =  "box" + str(box)
        req.link1_name  = 'link'       
        req.model2_name =  'ur5'       
        req.link2_name  = 'wrist_3_link'  

        self.gripper_control_off.call_async(req)      


    # function to attach    
    def attach_gripper(self, box):
            

        while not self.gripper_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EEF service not available, waiting again...')

        print(" IN THE MIDDLE OF ATTACH GRIPPER")    

        req = AttachLink.Request()
        req.model1_name =  "box" + str(box)
        req.link1_name  = 'link'       
        req.model2_name =  'ur5'       
        req.link2_name  = 'wrist_3_link'  

        self.gripper_control.call_async(req)


def main():
    
    rclpy.init()
    # LATER MIGHT GRAB FROM TASK 1B script
    box_ids = [1, 3, 49]
    
    # Create node for this example
    node = Task1C(box_ids=box_ids)
    
    node.declare_parameter(
            "filepath",
            "",
        )
    node.declare_parameter(
        "action",
        "add",
    )


    filepath = node.get_parameter("filepath").get_parameter_value().string_value
    action = node.get_parameter("action").get_parameter_value().string_value
    mesh_id = path.basename(filepath).split(".")[0]
    
    rackpos=[0.25,-0.62,0.17]
    rackqua=[0.0,0.0,0.706,0.708]#0.000003,-0.000040,1.571833
    box1_pos=[0.20, -0.65, 0.60]
    box1_qua=[0.000242,0.000000,0.707,0.707]
    box2_pos=[0.70, -0.23, -0.20]
    box2_qua=[0.71034, 0.70381, -0.002082, -0.0081301]
    box3_pos=[0.70, 0.49, -0.20]
    box3_qua=[0.71034, 0.70381, -0.002082, -0.0081301]
    d1=[ -0.69, 0.10, 0.44]
    qd1=[-0.6837, 0.72651, 0.052767, -0.043971]
    box_id=['1','3','49']
    
    filepath1 = RACK_MESH
    filepath2 = BOX_MESH
    boxpos=[box1_pos,box2_pos,box3_pos]
    boxqua=[box1_qua,box2_qua,box3_qua]
    # # Spin the node in background thread(s)
    if "add" == action:
            # Add collision mesh
            node.get_logger().info(
                f"Adding collision mesh '{filepath1}' {{position: {list( rackpos)}, quat_xyzw: {list(rackqua)}}}"
            )
           
            # node.get_logger().info(
            #     f"Adding collision mesh '{filepath3}' {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
            # )
            print(ur5.base_link_name())
            node.moveit2.add_collision_mesh(
                filepath=filepath1, id="rack2", position=rackpos, quat_xyzw=rackqua, frame_id=ur5.base_link_name()
            )
            
            for j in range(3):
                node.get_logger().info(
                f"Adding collision mesh '{filepath2}' {{position: {list(boxpos[j])}, quat_xyzw: {list(boxqua[j])}}}"
            )
                node.moveit2.add_collision_mesh(
                    filepath=filepath2, id=box_id[j], position=boxpos[j], quat_xyzw=boxqua[j], frame_id=ur5.base_link_name()
                )
             
            
    else:
        # Remove collision mesh
        node.get_logger().info(f"Removing collision mesh with ID '{mesh_id}'")
        node.moveit2.remove_collision_mesh(id=mesh_id)
    
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor.spin()

    # rclpy.spin(node)
    # node.destroy_node()


    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()