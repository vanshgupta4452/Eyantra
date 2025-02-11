#!/usr/bin/env python3
"""
Example of moving to a pose goal.
`ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False`
"""

from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from linkattacher_msgs.srv import AttachLink, DetachLink

from geometry_msgs.msg import TwistStamped
import time
from os import path







RACK_MESH = path.join(
	path.dirname(path.realpath(__file__)), "assets", "rack.stl"
)
# ARM_BASE_MESH = path.join(
# 	path.dirname(path.realpath(__file__)), "assets", "arm_base.dae"
# )

BOX_MESH=path.join(
	path.dirname(path.realpath(__file__)), "assets", "box.stl"
)
class GripperControl(Node):
    def __init__(self):
        super().__init__('move_ur5') 
        self.twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.gripper_control = self.create_client(AttachLink, '/GripperMagnetON')
        self.gripper_control_off = self.create_client(DetachLink, '/GripperMagnetOFF')
        self.twist_msg = TwistStamped()
        self.callback_group = ReentrantCallbackGroup()
        
        # self.moveit2 = MoveIt2(
        #         node=self,
        #         joint_names=ur5.joint_names(),
        #         base_link_name=ur5.base_link_name(),
        #         end_effector_name=ur5.end_effector_name(),
        #         group_name=ur5.MOVE_GROUP_ARM,
        #         callback_group=self.callback_group,
        #         velocity_scaling_factor=0.8,  # Set velocity scaling here
        #         acceleration_scaling_factor=0.8 , )

        self.declare_parameter(
        "joint_positions0",
        [0.0, -2.39, 2.41, -3.16, -1.59, 3.14],
        )
        self.declare_parameter(
        "joint_positions1",
        [0.0, -1.972219686249898, -1.0471975802753372, -3.141592653589793, -1.5882534483405718, 3.141592653589793]
,
        
        )

        self.declare_parameter(
        "joint_positions2",
        [-0.10471975511965977, -1.8849555921538759, 1.7453292519943295, -3.001966313430247, -1.4660765716752369, 3.141592653589793],

        
        )

        self.declare_parameter(
        "joint_positions3",
        [-1.5707963267948966, -2.3876104167282426, 2.4085543677521746, -3.1590459461097367, -1.5707963267948966, 3.141592653589793],



        
        )

    
	
    

    # def attach_gripper(self, box_name):
    #     req =AttachLink.Request()
    #     req.model1_name = box_name
    #     req.link1_name = 'link'
    #     req.model2_name = 'ur5'
    #     req.link2_name = 'wrist_3_link'

    #     future = self.attach_client.call_async(req)
    #     rclpy.spin_until_future_complete(self, future)
    #     if future.result() is not None:
    #         self.get_logger().info(f'Gripper attached to {box_name}')
    #     else:
    #         self.get_logger().error(f'Failed to attach: {future.exception()}')

    
    def detach_gripper(self, box):
        

        while not self.gripper_control_off.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EEF service not available, waiting again...')

        print("kuch kuch hora hai")    

        req = DetachLink.Request()
        req.model1_name =  box      
        req.link1_name  = 'link'       
        req.model2_name =  'ur5'       
        req.link2_name  = 'wrist_3_link'  

        self.gripper_control_off.call_async(req)      
    
    def attach_gripper(self, box):
        

        while not self.gripper_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EEF service not available, waiting again...')

        print("kuch kuch hora hai")    

        req = AttachLink.Request()
        req.model1_name =  box      
        req.link1_name  = 'link'       
        req.model2_name =  'ur5'       
        req.link2_name  = 'wrist_3_link'  

        self.gripper_control.call_async(req)
def main():
    rclpy.init()
    grip=GripperControl()
    callback_group = ReentrantCallbackGroup()
    
    # Create node for this example
    # points = [[0.20, -0.47, 0.65], [-0.69, 0.10, 0.44], [0.75, 0.49, -0.05], [0.75, -0.23, -0.05], [-0.69, 0.10, 0.44]]
    # quat = [[-0.007430290337651968, -0.7046612501144409, 0.7094658613204956, -0.007449863478541374]]
    
    p1=[ 0.20, -0.47, 0.65 ]
    q1=[0.7076910118850276, 0.0037718698896891246,0.0007400916322284481,0.7065116113405758]
    p2=[0.75, 0.45, -0.05]
    q2=[0.71034, 0.70381, -0.002082, -0.0081301]
    p3=[0.75, -0.23, -0.05]
    q3=[0.71038, 0.70367, 0.012708, 0.0068683]
    d1=[ -0.69, 0.10, 0.44]
    qd1=[-0.6837, 0.72651, 0.052767, -0.043971]
    box_id=['1','3','49']
    points=[p1,p2,p3]
    quat=[q1,q2,q3]
    #  points=[p1,d1,initial_pose,p2,initial_pose,d1,initial_pose,p3,initial_pose,d1]
    # quat=[q1,qd1,initial_quad,q2,initial_quad,qd1,initial_quad,q3,initial_quad,qd1]
    # Initialize the gripper control node
    

    for i in range(len(points)):
        node = Node("ex_pose_goal")
        
        node.declare_parameter(
            "filepath",
            "",
        )
        node.declare_parameter(
            "action",
            "add",
        )
        # Declare parameters for position and orientation
        node.declare_parameter("position", points[i])
        node.declare_parameter("quat_xyzw", quat[i])
        node.declare_parameter("cartesian", True)

        # Create callback group that allows execution of callbacks in parallel
        callback_group = ReentrantCallbackGroup()

        # Create MoveIt 2 interface
        moveit2 = MoveIt2(
            node=node,
            joint_names=ur5.joint_names(),
            base_link_name=ur5.base_link_name(),
            end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM,
            callback_group=callback_group,
              # Increase this value for higher acceleration
             )
        

        
        # Spin the node in background thread(s)
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(node)
        executor_thread = Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        # Get parameters
        filepath = node.get_parameter("filepath").get_parameter_value().string_value
        action = node.get_parameter("action").get_parameter_value().string_value
        joint_positions0 = grip.get_parameter("joint_positions0").get_parameter_value().double_array_value
        joint_positions1 = grip.get_parameter("joint_positions1").get_parameter_value().double_array_value
        joint_positions2 = grip.get_parameter("joint_positions2").get_parameter_value().double_array_value
        joint_positions3 = grip.get_parameter("joint_positions3").get_parameter_value().double_array_value
        # joint_value=[joint_positions0,joint_positions1,joint_positions2,joint_positions3]
        position = node.get_parameter("position").get_parameter_value().double_array_value
        quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
        cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value
        mesh_id = path.basename(filepath).split(".")[0]
        rackpos=[0.25,-0.62,0.17]
        rackqua=[0.0,0.0,0.706,0.708]
        
        # rackpos=[0.52,0.05,0.17]
        # rackqua=[0.00,0.00,0.00,0.00]
       
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
        # filepath3=ARM_BASE_MESH
        if "add" == action:
            # Add collision mesh
            node.get_logger().info(
                f"Adding collision mesh '{filepath1}' {{position: {list( rackpos)}, quat_xyzw: {list(rackqua)}}}"
            )
           
            # node.get_logger().info(
            #     f"Adding collision mesh '{filepath3}' {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
            # )
            print(ur5.base_link_name())
            moveit2.add_collision_mesh(
                filepath=filepath1, id="rack2", position=rackpos, quat_xyzw=rackqua, frame_id=ur5.base_link_name()
            )
            # node.get_logger().info(
            #     f"Adding collision mesh '{filepath3}' {{position: {list( rack_base_pos)}, quat_xyzw: {list(rack_base_qua)}}}"
            # )
           
            # # node.get_logger().info(
            # #     f"Adding collision mesh '{filepath3}' {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
            # # )
            # print(ur5.base_link_name())
            # moveit2.add_collision_mesh(
            #     filepath=filepath1, id="rack2", position=rackpos, quat_xyzw=rackqua, frame_id=ur5.base_link_name()
            # )
            for j in range(3):
                node.get_logger().info(
                f"Adding collision mesh '{filepath2}' {{position: {list(boxpos[j])}, quat_xyzw: {list(boxqua[j])}}}"
            )
                moveit2.add_collision_mesh(
                    filepath=filepath2, id=box_id[j], position=boxpos[j], quat_xyzw=boxqua[j], frame_id=ur5.base_link_name()
                )
            # moveit2.add_collision_mesh(
            #     filepath=filepath3, id=mesh_id, position=position, quat_xyzw=quat_xyzw, frame_id=ur5.base_link_name()
            # )
            # if i%2==0:
            #     moveit2.move_to_configuration(joint_value[i])

            # else:
            #     moveit2.move_to_configuration(joint_positions1)    
            node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions0)}}}")
        else:
            # Remove collision mesh
            node.get_logger().info(f"Removing collision mesh with ID '{mesh_id}'")
            moveit2.remove_collision_mesh(id=mesh_id)
        # Move to pose
        node.get_logger().info(
            f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
        )
    #    if i==0:
    #       moveit2.move_to_configuration(joint_positions3)
        # else:  
        #   moveit2.move_to_configuration(joint_positions2)    
        # node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions0)}}}")
        # if i==0:
        #   moveit2.move_to_configuration(joint_positions3)
        # else:  
        #   moveit2.move_to_configuration(joint_positions2)
        # moveit2.wait_until_executed()
        
        moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
        print("hehhehehehehehehehehehehhehe")
        
        box="box"+str(box_id[i])
        print(box,"............")
        moveit2.wait_until_executed()
        # time.sleep(1)
        grip.attach_gripper(box)
        # time.sleep(1)
        # moveit2.move_to_configuration(joint_positions0)
        # time.sleep(1)
        moveit2.move_to_configuration(joint_positions1)
        moveit2.wait_until_executed()
        # time.sleep(1)
        grip.detach_gripper(box)
        moveit2.move_to_configuration(joint_positions2)
        
        
       

        # Attach the gripper to the box
        
        

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()