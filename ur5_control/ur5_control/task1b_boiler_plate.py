#!/usr/bin/python3
# -*- coding: utf-8 -*-

'''
*****************************************************************************************
*
*        		===============================================
*           		    Logistic coBot (LB) Theme (eYRC 2024-25)
*        		===============================================
*
*  This script should be used to implement Task 1B of Logistic coBot (LB) Theme (eYRC 2024-25).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		    task1b_boiler_plate.py
# Functions:
#			        [ Comma separated list of functions in this file ]
# Nodes:		    Add your publishing and subscribing node
#			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/aligned_depth_to_color/image_raw, /etc... ]


################### IMPORT MODULES #######################

#-0.40514; 0.00032263; 0.91425; 0.00072804

import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped, Quaternion
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image

##################### FUNCTION DEFINITIONS #######################

def calculate_rectangle_area(coordinates):
    '''
    Description:    Function to calculate area or detected aruco

    Args:
        coordinates (list):     coordinates of detected aruco (4 set of (x,y) coordinates)

    Returns:
        area        (float):    area of detected aruco
        width       (float):    width of detected aruco
    '''

    ############ Function VARIABLES ############

    # You can remove these variables after reading the instructions. These are just for sample.
    print(coordinates)
    height = coordinates[0][1] - coordinates[2][1]
    width = coordinates[0][0] - coordinates[2][0]
    area = height * width

    ############ ADD YOUR CODE HERE ############

    # INSTRUCTIONS & HELP : 
    #	->  Recevice coordiantes from 'detectMarkers' using cv2.aruco library 
    #       and use these coordinates to calculate area and width of aruco detected.
    #	->  Extract values from input set of 4 (x,y) coordinates 
    #       and formulate width and height of aruco detected to return 'area' and 'width'.

    ############################################

    return area, width


def calculate_y_euler_angle(corners_3d):
    # Assuming corners_3d is a 4x3 array containing the 3D coordinates of the quadrilateral corners
    
    # Calculate two vectors in the plane of the quadrilateral
    v1 = corners_3d[1] - corners_3d[0]
    v2 = corners_3d[2] - corners_3d[0]
    
    # Calculate the normal to the plane (cross product of v1 and v2)
    normal_vector = np.cross(v1, v2)
    
    # Normalize the normal vector
    normal_vector = normal_vector / np.linalg.norm(normal_vector)
    
    # Project the normal vector onto the XZ plane (ignore Y component)
    projection_on_xz = np.array([normal_vector[0], 0, normal_vector[2]])
    
    # Normalize the projection
    projection_on_xz = projection_on_xz / np.linalg.norm(projection_on_xz)
    
    # The Y-axis Euler angle is the angle between the Z-axis and the projection on the XZ plane
    z_axis = np.array([0, 0, 1])
    
    # Compute the angle (in radians) between the Z-axis and the projection using the dot product
    angle_radians = np.arccos(np.dot(z_axis, projection_on_xz))
    
    # Convert to degrees
    angle_degrees = np.degrees(angle_radians)
    
    return angle_degrees


def calculate_z_euler_angle(corners_3d):
    # Calculate two vectors in the plane of the quadrilateral
    v1 = corners_3d[1] - corners_3d[0]
    v2 = corners_3d[2] - corners_3d[0]
    
    # Calculate the normal to the plane (cross product of v1 and v2)
    normal_vector = np.cross(v1, v2)
    
    # Normalize the normal vector
    normal_vector = normal_vector / np.linalg.norm(normal_vector)
    
    # Project the normal vector onto the XY plane (ignore Z component)
    projection_on_xy = np.array([normal_vector[0], normal_vector[1], 0])
    
    # Normalize the projection
    projection_on_xy = projection_on_xy / np.linalg.norm(projection_on_xy)
    
    # The Z-axis Euler angle is the angle between the X-axis and the projection on the XY plane
    x_axis = np.array([1, 0, 0])
    
    # Compute the angle (in radians) between the X-axis and the projection using the dot product
    angle_radians = np.arccos(np.dot(x_axis, projection_on_xy))
    
    # Convert to degrees
    angle_degrees = np.degrees(angle_radians)
    
    return angle_degrees


def calculate_x_euler_angle(corners_3d):
    v1 = corners_3d[1] - corners_3d[0]
    v2 = corners_3d[2] - corners_3d[0]
    
    normal_vector = np.cross(v1, v2)
    normal_vector = normal_vector / np.linalg.norm(normal_vector)
    
    projection_on_yz = np.array([0, normal_vector[1], normal_vector[2]])
    projection_on_yz = projection_on_yz / np.linalg.norm(projection_on_yz)
    
    y_axis = np.array([0, 1, 0])
    
    angle_radians = np.arccos(np.dot(y_axis, projection_on_yz))
    angle_degrees = np.degrees(angle_radians)
    
    return angle_degrees

def detect_aruco(image, depth):
    '''
    Description:    Function to perform aruco detection and return each detail of aruco detected 
                    such as marker ID, distance, angle, width, center point location, etc.

    Args:
        image                   (Image):    Input image frame received from respective camera topic

    Returns:
        center_aruco_list       (list):     Center points of all aruco markers detected
        distance_from_rgb_list  (list):     Distance value of each aruco markers detected from RGB camera
        angle_aruco_list        (list):     Angle of all pose estimated for aruco marker
        width_aruco_list        (list):     Width of all detected aruco markers
        ids                     (list):     List of all aruco marker IDs detected in a single frame 
    '''

    ############ Function VARIABLES ############

    # ->  You can remove these variables if needed. These are just for suggestions to let you get started

    # Use this variable as a threshold value to detect aruco markers of certain size.
    # Ex: avoid markers/boxes placed far away from arm's reach position  
    aruco_area_threshold = 1500

    # The camera matrix is defined as per camera info loaded from the plugin used. 
    # You may get this from /camer_info topic when camera is spawned in gazebo.
    # Make sure you verify this matrix once if there are calibration issues.
    cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])

    # The distortion matrix is currently set to 0. 
    # We will be using it during Stage 2 hardware as Intel Realsense Camera provides these camera info.
    dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])

    # We are using 150x150 aruco marker size
    size_of_aruco_m = 0.15

    # You can remove these variables after reading the instructions. These are just for sample.
    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids = []
 
    # print("image: ", image)
    img_grayscale = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()

    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    corners, ids, _ = aruco_detector.detectMarkers(img_grayscale)

    print("Detected markers:", ids)
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        
        for i in range(len(ids)):
            corner = corners[i][0]
            area, width = calculate_rectangle_area(corner)
            print("area: ", area)
            if area <= aruco_area_threshold:
                width_aruco_list.append(width)

                center = np.mean(corner, axis=0)
                center_aruco_list.append(center)
                # print("center:", center)
                # print("corner:", corner)
                # print("i:", i)


                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], size_of_aruco_m, cam_mat, dist_mat) # For a single marker
            else:    
                width_aruco_list.append(width)

                center = np.mean(corner, axis=0)
                center_aruco_list.append(center)
                # print("center:", center)
                # print("corner:", corner)
                # print("i:", i)


                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], size_of_aruco_m, cam_mat, dist_mat) # For a single marker
            # dist = np.sqrt(np.sum(np.square(tvec)))
            if depth is not None:
                dist = depth[int(center[1])][int(center[0])]/1000

                print("distance: ", dist)

                distance_from_rgb_list.append(dist)

                x1 = corner[0][0]
                y1 = corner[0][1]
                z1 = depth[int(y1)][int(x1)] / 1000

                x2 = corner[1][0]
                y2 = corner[1][1]
                z2 = depth[int(y2)][int(x2)] / 1000

                x3 = corner[2][0]
                y3 = corner[2][1]
                z3 = depth[int(y3)][int(x3)] / 1000

                x4 = corner[3][0]
                y4 = corner[3][1]
                z4 = depth[int(y4)][int(x4)] / 1000

                corners_3d = np.array([[x1, y1, z1], [x2, y2, z2], [x3, y3, z3], [x4, y4, z4]])
                y_euler_angle = calculate_y_euler_angle(corners_3d)
                z_euler_angle = calculate_z_euler_angle(corners_3d)
                x_euler_angle = calculate_x_euler_angle(corners_3d)

                angles = [x_euler_angle, y_euler_angle, z_euler_angle]

                angle_aruco_list.append(angles)

                # print("ANGLE of", ids[i], z_euler_angle)
                
                transformation_matrix = np.array([
                    [1, 0, 0],
                    [0, 0, -1],
                    [0, 1, 0]
                ]) 

                rotation_matrix, _ = cv2.Rodrigues(rvec)
                final = np.dot(rotation_matrix, transformation_matrix)
                r = R.from_matrix(final)
                
                # Get Euler angles: roll (x-axis), pitch (y-axis), yaw (z-axis)
                euler_angles_rvec = r.as_euler('xyz', degrees=True)  # Return in degrees
                print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
                print("RVEC: ", euler_angles_rvec)
                print("OURS: ", angles)
                print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")


            # z = euler_angles[2]
            # y = euler_angles[1]
            # x = euler_angles[0]


            cv2.drawFrameAxes(image, cam_mat, dist_mat, rvec, tvec, 0.15)

    # cv2.imshow('Detected Markers', image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    ############ ADD YOUR CODE HERE ############

    # INSTRUCTIONS & HELP : 

    #	->  Convert input BGR image to GRAYSCALE for aruco detection

    #   ->  Use these aruco parameters-
    #       ->  Dictionary: 4x4_50 (4x4 only until 50 aruco IDs)

    #   ->  Detect aruco marker in the image and store 'corners' and 'ids'
    #       ->  HINT: Handle cases for empty markers detection. 

    #   ->  Draw detected marker on the image frame which will be shown later

    #   ->  Loop over each marker ID detected in frame and calculate area using function defined above (calculate_rectangle_area(coordinates))

    #   ->  Remove tags which are far away from arm's reach positon based on some threshold defined

    #   ->  Calculate center points aruco list using math and distance from RGB camera using pose estimation of aruco marker
    #       ->  HINT: You may use numpy for center points and 'estimatePoseSingleMarkers' from cv2 aruco library for pose estimation

    #   ->  Draw frame axes from coordinates received using pose estimation
    #       ->  HINT: You may use 'cv2.drawFrameAxes'

    ############################################

    return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids


##################### CLASS DEFINITION #######################
def euler_to_quaternion_with_matrix(euler_angles, rotation, seq='xyz'):
    """
    Converts Euler angles to a quaternion after multiplying by a rotation matrix.

    :param euler_angles: List or tuple of Euler angles [roll, pitch, yaw] in radians.
    :param rotation_matrix: 3x3 rotation matrix to multiply with.
    :param seq: Sequence of axes for rotations, default is 'xyz' (roll, pitch, yaw).
    :return: Quaternion [x, y, z, w]
    """
    # Convert Euler angles to a rotation matrix
    euler_rotation = R.from_euler(seq, euler_angles).as_matrix()

    # Multiply the two matrices
    final_matrix = np.dot(euler_rotation, rotation)

    # Convert the final matrix to a quaternion
    r = R.from_matrix(final_matrix)
    quaternion = r.as_quat()  # Returns in [x, y, z, w] format
    return quaternion
# class Quaternion():

#     def __init__(self, x=0, y=0, z=0):
        
#         self.x = float(x)
#         self.y = float(y)
#         self.z = np.sin(z/2)
#         self.w = np.cos(z/2)

def quaternion_to_euler(quaternion):
    # Create a Rotation object from the quaternion
    rotation = R.from_quat(quaternion)
    # Convert the rotation to Euler angles (in radians)
    euler_angles = rotation.as_euler('xyz', degrees=True)  # Change 'xyz' to your desired order
    return euler_angles

class aruco_tf(Node):
    '''
    ___CLASS___

    Description:    Class which servers purpose to define process for detecting aruco marker and publishing tf on pose estimated.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class aruco_tf
                        All classes have a function called __init__(), which is always executed when the class is being initiated.
                        The __init__() function is called automatically every time the class is being used to create a new object.
                        You can find more on this topic here -> https://www.w3schools.com/python/python_classes.asp
        '''

        super().__init__('aruco_tf_publisher')                                          # registering node

        ############ Topic SUBSCRIPTIONS ############

        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.2                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
        
        self.cv_image = None                                                            # colour raw image variable (from colorimagecb())
        self.depth_image = None                                                         # depth image variable (from depthimagecb())


    def depthimagecb(self, data):
        '''
        Description:    Callback function for aligned depth camera topic. 
                        Use this function to receive image depth data and convert to CV2 image

        Args:
            data (Image):    Input depth image frame received from aligned depth camera topic

        Returns:
        '''
        self.depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
        
        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 

        #	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT: You may use CvBridge to do the same

        ############################################


    def colorimagecb(self, data):
        '''
        Description:    Callback function for colour camera raw topic.
                        Use this function to receive raw image data and convert to CV2 image

        Args:
            data (Image):    Input coloured raw image frame received from image_raw camera topic

        Returns:
        '''
        
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 

        #	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT:   You may use CvBridge to do the same
        #               Check if you need any rotation or flipping image as input data maybe different than what you expect to be.
        #               You may use cv2 functions such as 'flip' and 'rotate' to do the same

        ############################################


    def process_image(self):
        '''
        Description:    Timer function used to detect aruco markers and publish tf on estimated poses.

        Args:
        Returns:
        '''

        ############ Function VARIABLES ############

        # These are the variables defined from camera info topic such as image pixel size, focalX, focalY, etc.
        # Make sure you verify these variable values once. As it may affect your result.
        # You can find more on these variables here -> http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640 
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375
        
        if self.cv_image is not None:
            center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids = detect_aruco(self.cv_image, self.depth_image)
            
            if ids is not None:
                for i in range(len(ids)):
                    # angle_aruco = angle_aruco_list[i][2]
                    # angle_aruco = angle_aruco_list[i]
                    # angle_aruco = np.deg2rad(angle_aruco)
                    angles = np.deg2rad(angle_aruco_list[i])
                    print("\n --------- \n", angles, "\n --------- \n")

                    # angle_aruco = (0.788*angle_aruco) - ((angle_aruco**2)/3160)
                    # angle_aruco += 0.15
                #     transformation_matrix = np.array([
                #     [0, 0, 1],
                #     [1, 0, 0],
                #     [0, 1, 0]
                # ]) 
                #     transformation_matrix = np.array([
                #     [1, 0, 0],
                #     [0, 0, -1],
                #     [0, 1, 0]
                # ]) 
                    transformation_matrix = np.eye(3)

                    # cam_corr = angle_aruco_list[i][2]
                    # cam_corr = np.deg2rad(cam_corr)
                    # cam_corr = 0

                    print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
                    print("id: ", ids[i])
                    print("angles: ", angles)
                    print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")

                    if round(angles[2]) == 0:
                        q = euler_to_quaternion_with_matrix([angles[0], angles[1]-1.152 , angles[2]], transformation_matrix) #1.152; 0.02; 1.2379
                    elif round(angles[2]) == 3:
                        q = euler_to_quaternion_with_matrix([angles[0], angles[1]+1.152 , angles[2]], transformation_matrix)
                    else:
                        q = euler_to_quaternion_with_matrix([angles[0]+0.77, angles[1], angles[2]], transformation_matrix)

                    cX = center_aruco_list[i][0]
                    cY = center_aruco_list[i][1]
                    distance_from_rgb = float(distance_from_rgb_list[i])
                    y = distance_from_rgb * (sizeCamX - cX - centerCamX) / focalX
                    z = distance_from_rgb * (sizeCamY - cY - centerCamY) / focalY
                    x = distance_from_rgb
                    
                    image = cv2.circle(self.cv_image, (int(cX), int(cY)), 10, (0,255,0), 5)
    
                    try:

                        t0 = self.tf_buffer.lookup_transform( "camera_link", "base_link", rclpy.time.Time())       # look up for the transformation between 'obj_1' and 'base_link' frames
                        # cam_corr = cam_base_tf.transform.rotation.y

                        # self.get_logger().info(f'Successfully received data!')

                        t1 = TransformStamped()
                        t1.header.stamp = self.get_clock().now().to_msg()
                        t1.child_frame_id = "cam_" + str(ids[i])
                        t1.header.frame_id = "camera_link"

                        t1.transform.translation.x = x
                        t1.transform.translation.y = y
                        t1.transform.translation.z = z
                        
                        t1.transform.rotation.x = q[0]
                        t1.transform.rotation.y = q[1]
                        t1.transform.rotation.z = q[2]
                        t1.transform.rotation.w = q[3]
    
                        self.br.sendTransform(t1)

                        t_ = self.tf_buffer.lookup_transform("base_link", "cam_" + str(ids[i]), rclpy.time.Time())

                        t2 = TransformStamped()
                        t2.header.stamp = self.get_clock().now().to_msg()
                        t2.header.frame_id = "base_link"
                        t2.child_frame_id = "obj_" + str(ids[i][0])
                        t2.transform = t_.transform

                        self.br.sendTransform(t2)

                    except tf2_ros.TransformException as e:
                        self.get_logger().info(f'Could not transform base_link to object: {e}')
                        return
            #   ->  Then finally lookup transform between base_link and obj frame to publish the TF
            # #       You may use 'lookup_transform' function to pose of obj frame w.r.t base_link 
                    self.get_logger().info(f'Translation X:  {t2.transform.translation.x}')
                    self.get_logger().info(f'Translation Y:  {t2.transform.translation.y}')
                    self.get_logger().info(f'Translation Z:  {t2.transform.translation.z}')
                    self.get_logger().info(f'Rotation X:  {t2.transform.rotation.x}')                                # NOTE: rotations are in quaternions
                    self.get_logger().info(f'Rotation Y:  {t2.transform.rotation.y}')
                    self.get_logger().info(f'Rotation Z:  {t2.transform.rotation.z}')
                    self.get_logger().info(f'Rotation W:  {t2.transform.rotation.w}')
                    
                cv2.imshow("image", image)
                cv2.waitKey(1)
                # cv2.destroyAllWindows()

    def quaternion_multiply(self, q1, q2):
        """Multiply two quaternions (q1 * q2)"""
        w1, x1, y1, z1 = q1.w, q1.x, q1.y, q1.z
        w2, x2, y2, z2 = q2.w, q2.x, q2.y, q2.z
        q = Quaternion()
        q.x = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        q.y = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        q.z = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        q.w = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        return q


        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 

        #	->  Get aruco center, distance from rgb, angle, width and ids list from 'detect_aruco_center' defined above

        #   ->  Loop over detected box ids received to calculate position and orientation transform to publish TF 

        #   ->  Use this equation to correct the input aruco angle received from cv2 aruco function 'estimatePoseSingleMarkers' here
        #       It's a correction formula- 
        #       angle_aruco = (0.788*angle_aruco) - ((angle_aruco**2)/3160)

        #   ->  Then calculate quaternions from roll pitch yaw (where, roll and pitch are 0 while yaw is corrected aruco_angle)

        #   ->  Use center_aruco_list to get realsense depth and log them down. (divide by 1000 to convert mm to m)

        #   ->  Use this formula to rectify x, y, z based on focal length, center value and size of image
        #       x = distance_from_rgb * (sizeCamX - cX - centerCamX) / focalX
        #       y = distance_from_rgb * (sizeCamY - cY - centerCamY) / focalY
        #       z = distance_from_rgb
        #       where, 
        #               cX, and cY from 'center_aruco_list'
        #               distance_from_rgb is depth of object calculated in previous step
        #               sizeCamX, sizeCamY, centerCamX, centerCamY, focalX and focalY are defined above

        #   ->  Now, mark the center points on image frame using cX and cY variables with help of 'cv2.cirle' function 

        #   ->  Here, till now you receive coordinates from camera_link to aruco marker center position. 
        #       So, publish this transform w.r.t. camera_link using Geometry Message - TransformStamped 
        #       so that we will collect it's position w.r.t base_link in next step.
        #       Use the following frame_id-
        #           frame_id = 'camera_link'
        #           child_frame_id = 'cam_<marker_id>'          Ex: cam_20, where 20 is aruco marker ID

        #   ->  Then finally lookup transform between base_link and camera_link to publish the TF
        #       You may use 'lookup_transform' function to pose of cam frame w.r.t base_link 

        #   ->  And now publish TF between object frame and base_link
        #       Use the following frame_id-
        #           frame_id = 'base_link'
        #           child_frame_id = 'obj_<marker_id>'          Ex: obj_20, where 20 is aruco marker ID

        #   ->  At last show cv2 image window having detected markers drawn and center points located using 'cv2.imshow' function.
        #       Refer MD book on portal for sample image -> https://portal.e-yantra.org/

        #   ->  NOTE:   The Z axis of TF should be pointing inside the box (Purpose of this will be known in task 1C)
        #               Also, auto eval script will be judging angular difference as well. So, make sure that Z axis is inside the box (Refer sample images on Portal - MD book)

        ############################################


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('aruco_tf_process')                    # creating ROS node

    node.get_logger().info('Node created: Aruco tf process')        # logging information

    aruco_tf_class = aruco_tf()                                     # creating a new object for class 'aruco_tf'

    rclpy.spin(aruco_tf_class)                                      # spining on the object to make it alive in ROS 2 DDS

    aruco_tf_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':
    '''
    Description:    If the python interpreter is running that module (the source file) as the main program, 
                    it sets the special __name__ variable to have a value “__main__”. 
                    If this file is being imported from another module, __name__ will be set to the module’s name.
                    You can find more on this here -> https://www.geeksforgeeks.org/what-does-the-if-__name__-__main__-do/
    '''

    main()