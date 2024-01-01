# -*- coding: utf-8 -*-
"""
@Description :   Create URDF dataset (index, path, object name, size, fingerprint, etc.)
@Author      :   Yan Ding 
@Time        :   2023/12/30 20:43:44
"""

import os, sys
import time
import pybullet as p
import pybullet_data
import random
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
import pickle
from datetime import datetime
from utils import Client, find_urdf_files_relative, create_excel, euler_to_quaternion
import pandas, openpyxl
import math

# ----------------------------------------------------------------
# Get path
# ----------------------------------------------------------------
utils_path = os.path.join(os.getcwd(), 'utils.py')
folder_path = os.path.dirname(utils_path)
print(f'utils path: {utils_path}')
print(f'folder path: {folder_path}')

# ----------------------------------------------------------------
# Initialize pybullet GUI
# ----------------------------------------------------------------
demo = Client()
demo.enable_vertical_view(5.0, [1.9, 0, -4], -90, -65)

# ----------------------------------------------------------------
# Load table
# ----------------------------------------------------------------
# table_path = folder_path + '/furniture_table_rectangle/table.urdf'
# table_pose = [1.0, 0.0, 0.0]
# table_orientation = [0, 0, 0.7071, 0.7071]
# table_id = demo.add_table(table_path, table_pose, table_orientation)
# [min_x, min_y, min_z], [max_x, max_y, max_z] = demo.get_bounding_box(table_id)
# table_height = max_z + 0.05
# demo.run(10) # simulate 10 step

# ----------------------------------------------------------------
# (option) Load a specific object
# ----------------------------------------------------------------
# object_path = folder_path + '/black_marker/model.urdf'
# object_pose = [1.0, 0.0, table_height]
# object_orientation = [0, 0, 0.7071, 0.7071]
# demo.add_object(object_path, object_pose, object_orientation)
# demo.run(10) # simulate 10 step

# ----------------------------------------------------------------
# Get all urdf objects
# ----------------------------------------------------------------
output_path = 'urdf_dataset.xlsx'
subfolders, urdf_paths = find_urdf_files_relative(folder_path)
print(f'subfolders: {subfolders}, urdf_paths: {urdf_paths}')

# ----------------------------------------------------------------
# Load one object automatically
# ----------------------------------------------------------------
urdf_sizes = []
for index in range(23, len(urdf_paths)):
    print('-' * 30)
    print(f'index: {index}')

    object_path = folder_path + '/' + urdf_paths[index]
    # object_pose = [1.0, 0.0, table_height]
    object_pose = [0.0, 0.0, 0]

    euler_object_orientation = [0, 0, 0]
    print(f'euler_object_orientation: {euler_object_orientation}')

    quaternion_object_orientation = euler_to_quaternion(euler_object_orientation[0], euler_object_orientation[1], euler_object_orientation[2])
    print(f'quaternion_object_orientation: {quaternion_object_orientation}')

    object_id = demo.add_object(object_path, object_pose, quaternion_object_orientation)
    [min_x, min_y, min_z], [max_x, max_y, max_z] = demo.get_bounding_box(object_id)
    size_x, size_y, size_z = abs(max_x - min_x), abs(max_y - min_y), abs(max_z - min_z)
    urdf_sizes.append([size_x, size_y, size_z])
    print(f'size_x: {size_x}, size_y: {size_y}, size_z: {size_z}')
    
    input("Press Enter to continue...")  # Wait for user input
    # demo.wait(60)
    demo.remove_object(object_id)

# ----------------------------------------------------------------
# Create excel file
# ----------------------------------------------------------------
# create_excel(subfolders, urdf_paths, urdf_sizes, output_path)

# ----------------------------------------------------------------
# Get scene image
# ----------------------------------------------------------------
# color, depth, segment = demo.render_image()
# demo.save_img(color, 'scene.png', folder_path)

# ----------------------------------------------------------------
# Get object image
# ----------------------------------------------------------------
# color, depth, segment = demo.render_image()
# demo.save_img(color, 'scene.png', folder_path)

# demo.run(100000) # simulate 10 step
demo.disconnect_pybullet()