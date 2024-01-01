# -*- coding: utf-8 -*-
"""
@Description :   A few functions used in bestman robot, where the robot has a base and an arm.
@Author      :   Yan Ding 
@Time        :   2023/12/30 20:43:44
"""

import os, sys
import time
import pybullet
import pybullet_data
from urdf_models import models_data
import random
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
import pickle
from datetime import datetime
from PIL import Image
import pandas as pd
import math

class Client():
    def __init__(self):
        # pybullet.connect(pybullet.DIRECT) # pybullet.GUI for local GUI.
        pybullet.connect(pybullet.GUI) # pybullet.GUI for local GUI.
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        pybullet.setGravity(0, 0, -9.8)

        # self.plane_id = pybullet.loadURDF("plane.urdf")
        self.frequency = 120

        # Initialize camera parameters as instance variables
        self.cam_width, self.cam_height = 480, 480
        self.cam_target_pos = [0.0, 0.0, 0.5]
        self.cam_distance = 1.5
        self.cam_yaw, self.cam_pitch, self.cam_roll = -90, -90, 0
        self.cam_up, self.cam_up_axis_idx, self.cam_near_plane, self.cam_far_plane, self.cam_fov = [0, 0, 1], 2, 0.01, 100, 60

        self.cam_view_matrix = pybullet.computeViewMatrixFromYawPitchRoll(
            self.cam_target_pos, 
            self.cam_distance, 
            self.cam_yaw, 
            self.cam_pitch, 
            self.cam_roll, 
            self.cam_up_axis_idx
        )
        self.cam_projection_matrix = pybullet.computeProjectionMatrixFOV(
            self.cam_fov, 
            self.cam_width / self.cam_height, 
            self.cam_near_plane, 
            self.cam_far_plane
        )
        self.znear, self.zfar = 0.01, 10.
        print(f'Camera information:\ncam_width: {self.cam_width}, cam_height: {self.cam_height}, cam_target_pos: {self.cam_target_pos}, cam_yaw: {self.cam_yaw}, cam_pitch: {self.cam_pitch}, cam_roll: {self.cam_roll}')


    def enable_vertical_view(self, dist, position, yaw=0, pitch=-89.9):
        """
        Set the debug visualizer camera in a vertical view.
        
        Args:
            dist (float): The distance of the camera from the target point.
            position (list): A list of three floats representing the target position in 3D space.
            yaw (float, optional): The yaw component of the camera orientation. Defaults to 0.
            pitch (float, optional): The pitch component of the camera orientation. Defaults to -89.9.
        """
        pybullet.resetDebugVisualizerCamera(
            cameraDistance=dist,
            cameraYaw=yaw,
            cameraPitch=pitch,
            cameraTargetPosition=position,
        )

    def render_image(self):
        # get raw data
        _, _, color, depth, segment = pybullet.getCameraImage(
            width=self.cam_width,
            height=self.cam_height,
            viewMatrix=self.cam_view_matrix,
            projectionMatrix=self.cam_projection_matrix,
            shadow=1,
            flags=pybullet.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX,
            renderer=pybullet.ER_BULLET_HARDWARE_OPENGL
        )

        # get color image.
        color_image_size = (self.cam_width, self.cam_height, 4)
        color = np.array(color, dtype=np.uint8).reshape(color_image_size)
        color = color[:, :, :3]  # remove alpha channel

        # get depth image.
        depth_image_size = (self.cam_width, self.cam_height)
        zbuffer = np.float32(depth).reshape(depth_image_size)
        depth = (self.zfar + self.znear - (2 * zbuffer - 1) * (self.zfar - self.znear))
        depth = (2 * self.znear * self.zfar) / depth

        # get segment image.
        segment = np.reshape(segment, [self.cam_width, self.cam_height]) * 1. / 255.
        return color, depth, segment

    def save_img(self, vector, img_name, img_path):
        image = Image.fromarray(vector)
        image.save(img_path + '/' + img_name)
        print(f'image is saved at {img_path}/{img_name}')

    def add_table(self, urdf_path, urdf_pose=[1.0, 0.0, 0.0], urdf_orientation=[0, 0, 0.7071, 0.7071]):
        if not os.path.exists(urdf_path):
            print(f'Error: cannot find {urdf_path}!')
            sys.exit(1)
        table_id = pybullet.loadURDF(urdf_path, basePosition=urdf_pose, baseOrientation=urdf_orientation)
        return table_id

    def add_object(self, urdf_path, urdf_pose, urdf_orientation):
        if not os.path.exists(urdf_path):
            print(f'Error: cannot find {urdf_path}!')
            sys.exit(1)
        object_id = pybullet.loadURDF(urdf_path, basePosition=urdf_pose, baseOrientation=urdf_orientation)
        return object_id

    def get_bounding_box(self, obj_id):
        (min_x, min_y, min_z), (max_x, max_y, max_z)= pybullet.getAABB(obj_id)
        return [min_x, min_y, min_z], [max_x, max_y, max_z]

    def draw_aabb(self, object_id):
        collision_distance = -0.01
        aabb = inflate_aabb(pybullet.getAABB(object_id), collision_distance)
        aabb_min = aabb[0]
        aabb_max = aabb[1]
        corners = [
            [aabb_min[0], aabb_min[1], aabb_min[2]],  # 0
            [aabb_max[0], aabb_min[1], aabb_min[2]],  # 1
            [aabb_max[0], aabb_max[1], aabb_min[2]],  # 2
            [aabb_min[0], aabb_max[1], aabb_min[2]],  # 3
            [aabb_min[0], aabb_min[1], aabb_max[2]],  # 4
            [aabb_max[0], aabb_min[1], aabb_max[2]],  # 5
            [aabb_max[0], aabb_max[1], aabb_max[2]],  # 6
            [aabb_min[0], aabb_max[1], aabb_max[2]],  # 7
        ]
        lines = [
            (0, 1),
            (1, 2),
            (2, 3),
            (3, 0),  # bottom face
            (4, 5),
            (5, 6),
            (6, 7),
            (7, 4),  # top face
            (0, 4),
            (1, 5),
            (2, 6),
            (3, 7),  # vertical edges
        ]
        color = [1, 0, 0]
        life_time = 0.05
        for line in lines:
            pybullet.addUserDebugLine(
                lineFromXYZ=corners[line[0]],
                lineToXYZ=corners[line[1]],
                lineColorRGB=color,
                lineWidth=2,
                lifeTime=life_time,
            )

    def wait(self, x): # seconds
        time.sleep(x)
        print("-" * 20 + "\n" + "Has waitted {} seconds".format(x))

    def run(self, x): # steps
        for _ in range(x):
            pybullet.stepSimulation()
            time.sleep(1.0 / self.frequency)
        print("-" * 20 + "\n" + "Has runned {} simulation steps".format(x))
    
    def remove_object(self, object_id):
        pybullet.removeBody(object_id)

    def disconnect_pybullet(self):
        pybullet.disconnect()
        print("-" * 20 + "\n" + "The script ends!"+ "\n" + "-" * 20)

# ----------------------------------------------------------------
# Get urdf info
# ----------------------------------------------------------------
def find_urdf_files_relative(folder_path):
    """
    Finds URDF files in a given directory and returns their relative paths
    along with the subfolder names they reside in. The URDF files are sorted
    by their relative paths while maintaining their association with subfolder names.

    Parameters:
    folder_path (str): Path to the directory containing URDF files.

    Returns:
    list, list: Two lists, one containing subfolder names and the other containing
                relative paths of URDF files.
    """
    urdf_info = []  # Will contain tuples of (subfolder_name, urdf_file_path)
    for root, dirs, files in os.walk(folder_path):
        for file in files:
            if file.endswith('.urdf'):
                # Compute the relative path
                relative_path = os.path.relpath(os.path.join(root, file), folder_path)
                # Get the subfolder name
                subfolder_name = os.path.relpath(root, folder_path)
                # Add subfolder name and URDF file path as a tuple
                urdf_info.append((subfolder_name, relative_path))

    # Sort the list based on URDF file paths
    urdf_info.sort(key=lambda x: x[1])

    # Separate the subfolder names and URDF file paths
    subfolders, urdf_files = zip(*urdf_info)  # This creates two tuples

    return list(subfolders), list(urdf_files)  # Convert tuples to lists

def create_excel(subfolder_names, urdf_files, urdf_sizes, output_path):
    """
    Creates an Excel file with URDF file information.

    Parameters:
    subfolder_names (list): List of names of subfolders containing URDF files.
    urdf_files (list): List of relative paths of URDF files.
    urdf_sizes (list): List of sizes for each URDF file, each size is a tuple (size_x, size_y, size_z).
    output_path (str): Path where the Excel file will be saved.
    """
    # Ensure the lists have the same length
    if not (len(subfolder_names) == len(urdf_files) == len(urdf_sizes)):
        raise ValueError("All lists must have the same length.")

    # Create a DataFrame with the provided information
    df = pd.DataFrame({
        'subfolder_name': subfolder_names,
        'relative_path': urdf_files,
        'size_x': [size[0] for size in urdf_sizes],
        'size_y': [size[1] for size in urdf_sizes],
        'size_z': [size[2] for size in urdf_sizes],
        'object_name': subfolder_names,
    })
    df.index.name = 'index'

    # Export to Excel file
    df.to_excel(output_path, engine='openpyxl')

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles (in radians) to Quaternion.

    Parameters:
    roll (float): Rotation angle around X-axis (Roll)
    pitch (float): Rotation angle around Y-axis (Pitch)
    yaw (float): Rotation angle around Z-axis (Yaw)

    Returns:
    tuple: Quaternion (x, y, z, w)
    """
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

    return [qx, qy, qz, qw]