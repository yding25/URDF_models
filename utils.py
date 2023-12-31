# -*- coding: utf-8 -*-

import os
import time
import pybullet as p
import pybullet_data
from urdf_models import models_data
import random
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
import pickle
from datetime import datetime


class Client():
  def __init__(self):
    pybullet.connect(pybullet.DIRECT) # pybullet.GUI for local GUI.
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    pybullet.setGravity(0, 0, -9.8)

    self.plane_id = pybullet.loadURDF("plane.urdf")
    
    # camera width and height
    self.cam_width = 480
    self.cam_height = 480

  def render_image(self):
    # camera parameters
    cam_target_pos = [1.0, 0.0, 0.5]
    cam_distance = 1.5
    cam_yaw, cam_pitch, cam_roll = -90, -90, 0
    cam_up, cam_up_axis_idx, cam_near_plane, cam_far_plane, cam_fov = [0, 0, 1], 2, 0.01, 100, 60
    cam_view_matrix = pybullet.computeViewMatrixFromYawPitchRoll(cam_target_pos, cam_distance, cam_yaw, cam_pitch, cam_roll, cam_up_axis_idx)
    cam_projection_matrix = pybullet.computeProjectionMatrixFOV(cam_fov, self.cam_width*1./self.cam_height, cam_near_plane, cam_far_plane)
    znear, zfar = 0.01, 10.

    # get raw data
    _, _, color, depth, segment = pybullet.getCameraImage(
        width=self.cam_width,
        height=self.cam_height,
        viewMatrix=cam_view_matrix,
        projectionMatrix=cam_projection_matrix,
        shadow=1,
        flags=pybullet.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX,
        renderer=pybullet.ER_BULLET_HARDWARE_OPENGL)

    # get color image.
    color_image_size = (self.cam_width, self.cam_height, 4)
    color = np.array(color, dtype=np.uint8).reshape(color_image_size)
    color = color[:, :, :3]  # remove alpha channel

    # get depth image.
    depth_image_size = (self.cam_width, self.cam_height)
    zbuffer = np.float32(depth).reshape(depth_image_size)
    depth = (zfar + znear - (2 * zbuffer - 1) * (zfar - znear))
    depth = (2 * znear * zfar) / depth

    # get segment image.
    segment = np.reshape(segment, [self.cam_width, self.cam_height]) * 1. / 255.
    return color, depth, segment


  def add_objects(self, utensil_name, utensil_pose):
    utensil_id = {}

    flags = pybullet.URDF_USE_INERTIA_FROM_FILE
    path = '/content/urdf_models/'

    if not os.path.exists(path):
      print('Error: cannot find /content/urdf_models/!')
      sys.exit(1)

    # add table
    table_id = pybullet.loadURDF("/content/urdf_models/furniture_table_rectangle/table.urdf", basePosition=[1.0, 0.0, 0.0], baseOrientation=[0, 0, 0.7071, 0.7071])
    utensil_id['table'] = table_id

  def get_bounding_box(self, obj_id):
    (min_x, min_y, min_z), (max_x, max_y, max_z)= pybullet.getAABB(obj_id)
    return [min_x, min_y, min_z], [max_x, max_y, max_z]

  def disconnect(self):
    pybullet.disconnect()

def simulate(M):
    for _ in range(240 * M):
        p.stepSimulation()
        time.sleep(1.0 / 240)


def world_to_image(physicsClient, viewMatrix, projectionMatrix, world_point):
    """
    Function to transform world coordinates to image coordinates
    """

    def reshape_matrix(matrix):
        """
        Reshape a 1D matrix of length 16 (list or numpy array) to a 4x4 2D matrix
        """
        return np.reshape(matrix, (4, 4))

    # Reshape the matrices
    viewMatrix = reshape_matrix(viewMatrix)
    projectionMatrix = reshape_matrix(projectionMatrix)

    # Project world to camera
    world_point = np.append(world_point, 1)  # Add homogenous coordinate
    view_projection_matrix = np.matmul(projectionMatrix, viewMatrix)
    camera_point = np.matmul(view_projection_matrix, world_point)

    # Normalize homogenous coordinates
    camera_point = camera_point[:3] / camera_point[3]

    # Convert camera coordinates to image coordinates
    img_width = 480
    img_height = 480
    img_point = np.zeros(2, dtype=int)
    img_point[0] = int((camera_point[0] + 1.0) / 2.0 * img_width)  # x
    img_point[1] = int((1.0 - camera_point[1]) / 2.0 * img_height)  # y

    return img_point


def crop_image(image, center, size):
    """
    Function to crop a square image at "center" with side length "size".
    "center" is a 2-element list or tuple or ndarray [x, y]
    "size" is an int
    """
    image_height, image_width = image.shape
    top = max(0, int(center[1] - size / 2))
    bottom = min(image_height, int(center[1] + size / 2))
    left = max(0, int(center[0] - size / 2))
    right = min(image_width, int(center[0] + size / 2))
    return image[top:bottom, left:right]


def generate_random_position_within_radius(radius):
    # Generate random angle
    theta = 2 * np.pi * np.random.uniform(0, 1)

    # Generate random radius
    r = radius * np.random.uniform(0, 1)

    # Convert polar coordinates to Cartesian coordinates
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    z = table_height

    return [x, y, z]


# ----------------------------------------------------------------
# Get depth image
# ----------------------------------------------------------------
def get_depth_image(name):
    img_arr = p.getCameraImage(
        480, 480, viewMatrix, projectionMatrix, shadow=1, lightDirection=[1, 1, 1]
    )
    depth_buffer = img_arr[3]  # depth buffer
    near = 0.01  # Near plane distance
    far = 100  # Far plane distance
    depth = far * near / (far - (far - near) * depth_buffer)
    # personalized colormap
    cdict = {
        "red": [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
        "green": [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
        "blue": [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
    }
    custom_cmap = LinearSegmentedColormap("custom_cmap", cdict)
    # plt.imshow(depth, cmap=custom_cmap)
    # plt.colorbar()
    # plt.show()

    # image_point = world_to_image(
    #     physicsClient, viewMatrix, projectionMatrix, [0, 0, 0.05]
    # )
    image_point = [240, 240]

    crop_size = 200
    depth_image = crop_image(depth, image_point, crop_size)
    # print("depth size:{} * {}".format(len(depth_image), len(depth_image[0])))
    # print("image_point:{}".format(image_point))
    # plt.imshow(depth_image, cmap=custom_cmap)
    # plt.imsave(name + '.png', depth_image, cmap=custom_cmap)
    # plt.colorbar()
    # plt.show()
    return depth_image


# ----------------------------------------------------------------
#  Check collision
# ----------------------------------------------------------------
def inflate_aabb(aabb, inflation):
    return (
        [coord - inflation for coord in aabb[0]],  # Lower limits
        [coord + inflation for coord in aabb[1]],  # Upper limits
    )


def collision_exists(object_ids, new_object_id, collision_distance=-0.01):
    # print('enter exist_collision, object_ids:{}'.format(object_ids))
    if len(object_ids) == 0:
        return False
    # Loop over all pairs of objects
    for object_x in object_ids:
        # Check for collision between object i and object j
        print('object_ids:{}, Check for collision between new object {} and object {}'.format(object_ids, new_object_id, object_x))
        aabb_x = inflate_aabb(p.getAABB(object_x), collision_distance)
        aabb_y = inflate_aabb(p.getAABB(new_object_id), collision_distance)
        if (
            aabb_x[0][0] <= aabb_y[1][0]
            and aabb_x[1][0] >= aabb_y[0][0]
            and aabb_x[0][1] <= aabb_y[1][1]
            and aabb_x[1][1] >= aabb_y[0][1]
            # and aabb_x[0][2] <= aabb_y[1][2]
            # and aabb_x[1][2] >= aabb_y[0][2]
        ):
            # print("Collision detected")
            return True
    return False


def draw_aabb(object_id):
    collision_distance = -0.01
    aabb = inflate_aabb(p.getAABB(object_id), collision_distance)
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
        p.addUserDebugLine(
            lineFromXYZ=corners[line[0]],
            lineToXYZ=corners[line[1]],
            lineColorRGB=color,
            lineWidth=2,
            lifeTime=life_time,
        )
