"""
main idea:
step 0: download YCB dataset

step 1.1 (get depth image A): first put multiple objects randomly within a area centered at [0, 0] and radius x
these objects are at different angles, different positons. but they do not have collisions with each other

step 1.2 (get depth image B): then put one random object in a fixed position [0, 0]

step 1.3 (get probability): repeat n_iterations times and get success rate

trainning data fromat: depth image A, depth image B, success rate (0.0~1.0)
"""

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


# ----------------------------------------------------------------
# create a folder to store trainning data
# ----------------------------------------------------------------
data_dir = "./data"
if not os.path.exists(data_dir):
    os.makedirs(data_dir)

# ----------------------------------------------------------------
# Initialize the GUI and others
# ----------------------------------------------------------------
enable_GUI = True
width, height = 480, 480
if enable_GUI:
    # physicsClient = p.connect(p.GUI)
    physicsClient = p.connect(p.GUI, options=f'--width={width} --height={height}')
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
else:
    physicsClient = p.connect(p.DIRECT)
p.setGravity(0, 0, -9.8)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetDebugVisualizerCamera(
    cameraDistance=2.0,
    cameraYaw=0,
    cameraPitch=-89.9,
    cameraTargetPosition=[0, 0, 0],
    physicsClientId=physicsClient,
)
flags = p.URDF_USE_INERTIA_FROM_FILE
current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
filename = "./data/" + "pc_" + current_time + ".pkl"
print('filename:{}'.format(filename))
video_name = "./data/" + "pc_" + current_time + ".mp4"
models = models_data.model_lib()
namelist = models.model_name_list

text_position = [-0.5, 0.5, 0.75]
text_size = 2  # font size
text_color = [0, 0, 1]
# text_id = p.addUserDebugText("test", text_position, textColorRGB=text_color, textSize=text_size)
# p.removeUserDebugItem(text_id)

# ----------------------------------------------------------------
# Load plan and table
# ----------------------------------------------------------------
p.loadURDF("plane.urdf")
tableStartPos = [0, 0, 0]
tableStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
table_id = p.loadURDF("table/table.urdf", tableStartPos, tableStartOrientation)
(_, _, _), (_, _, table_height) = p.getAABB(table_id)
table_height += 0.08

# ----------------------------------------------------------------
# Set camera parameters
# ----------------------------------------------------------------
tablePos, _ = p.getBasePositionAndOrientation(table_id)
cameraPos = [tablePos[0], tablePos[1], table_height + 0.7]
cameraUp = [0, 1, 0]  # bird-view
viewMatrix = p.computeViewMatrix(cameraPos, tablePos, cameraUp)
# projectionMatrix = p.computeProjectionMatrixFOV(
#     fov=60, aspect=1.0, nearVal=0.01, farVal=100.0
# )

def ortho_matrix(left, right, bottom, top, near, far):
    return [
        [2 / (right - left), 0, 0, -(right + left) / (right - left)],
        [0, 2 / (top - bottom), 0, -(top + bottom) / (top - bottom)],
        [0, 0, -2 / (far - near), -(far + near) / (far - near)],
        [0, 0, 0, 1],
    ]
p.resetDebugVisualizerCamera(
            cameraDistance=1.0,
            cameraYaw=0.0,
            cameraPitch=-89.90,
            cameraTargetPosition=[0, 0, 0],
        )
left = -0.5
right = 0.5
bottom = -0.5
top = 0.5
near = 0.01
far = 10
projectionMatrix = ortho_matrix(left, right, bottom, top, near, far)

# ----------------------------------------------------------------
# Objects
# ----------------------------------------------------------------
object_names = ["blue_cup", "blue_plate", "plastic_apple", "plastic_peach"]
# object_names = ["blue_cup"]

n_iterations = 1 # iteration
n_trials = 1 # used for calculating success rate

for iter in range(n_iterations):
    # ----------------------------------------------------------------
    # Initialize list to store object ids
    # ----------------------------------------------------------------
    object_ids = []
    positions_orientations = []

    # ----------------------------------------------------------------
    # Generate random positions for objects within the specified radius
    # ----------------------------------------------------------------
    n_objects = np.random.randint(0, len(object_names) + 1)
    for _ in range(n_objects):
        object_name = random.choice(object_names) # Choose a random object
        num_c = 0
        while True:
            position_other = generate_random_position_within_radius(0.25)
            orientation_other = p.getQuaternionFromEuler([0, 0, np.random.uniform(0, 2 * np.pi)])
            # orientation_other = p.getQuaternionFromEuler([0, 0, 0])
            object_id = p.loadURDF(models[object_name], position_other, orientation_other)
            # simulate(1)
            draw_aabb(object_id)
            print('-' * 20 + '\n' + 'object_id:{} is waitting for to be added.'.format(object_id))

            # ----------------------------------------------------------------
            # Ensure that the position is free of other objects
            # ----------------------------------------------------------------
            if not collision_exists(object_ids, object_id):
                # No collision detected, add the object and exit the loop
                positions_orientations.append([object_name, position_other, orientation_other])
                object_ids.append(object_id)
                print('object_id:{} has been added.'.format(object_id))
                print("object_name:{} position:{}".format(object_name, position_other))
                break
            else:
                # Collision detected, remove the object and try again
                p.removeBody(object_id)
                simulate(2)
                print('-' * 20 + '\n' + 'object_id:{} has been removed.'.format(object_id))
            num_c += 1
            if num_c >= 1000:
                print("Try many times, exit!")
                break
    # Get depth image A
    simulate(2)
    print("-" * 20 + "\n" + "Image A")
    depth_image_A = get_depth_image("depth_image_A")
    # Remove all objects from the scene before getting depth image B
    for object_id in object_ids:
        p.removeBody(object_id)

    # ----------------------------------------------------------------
    # Generate a random position for the single object at [0, 0]
    # ----------------------------------------------------------------
    object_name = random.choice(object_names)
    position_ego = [0, 0, table_height]
    orientation_ego = p.getQuaternionFromEuler([0, np.pi, np.random.uniform(0, 2 * np.pi)]) #reverse object
    single_object_id = p.loadURDF(models[object_name], position_ego, orientation_ego)
    orientation_ego = p.getQuaternionFromEuler([0, 0, np.random.uniform(0, 2 * np.pi)])
    simulate(1)
    # Get depth image B
    simulate(2)
    print("-" * 20 + "\n" + "Image B")
    depth_image_B = get_depth_image("depth_image_B")
    p.removeBody(single_object_id)

    enable_plot = False
    if enable_plot:
        # Personalized colormap
        cdict = {
            "red": [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
            "green": [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
            "blue": [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
        }
        custom_cmap = LinearSegmentedColormap("custom_cmap", cdict)

        plt.imshow(depth_image_A, cmap=custom_cmap)
        plt.title("depth_image_A")
        plt.colorbar()
        plt.show()

        plt.imshow(depth_image_B, cmap=custom_cmap)
        plt.title("depth_image_B")
        plt.colorbar()
        plt.show()

    # ----------------------------------------------------------------
    # Simulate and get success rate
    # ----------------------------------------------------------------
    print("-" * 20 + "\n" + "Start n_trials!")
    # logId = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, video_name)
    if n_objects > 0:
        success_count = 0
        for _ in range(n_trials):
            object_ids = []
            initial_positions_other = []  # initial location
            # Load the object in image A
            for item in positions_orientations:
                object_id = p.loadURDF(models[item[0]], item[1], item[2])
                # simulate(1)
                object_ids.append(object_id)
                initial_positions_other.append(p.getBasePositionAndOrientation(object_id)[0]) # 记录初始位置
                simulate(2)
            
            position_c_ego = position_ego.copy()
            # position_c_ego[0] += random.uniform(-0.02, 0.02)
            # position_c_ego[1] += random.uniform(-0.02, 0.02)
            position_c_ego[2] += 0.05
            # Load the object in image B
            target_object_id = p.loadURDF(models[object_name], position_c_ego, orientation_ego)
            simulate(1)
            object_ids.append(target_object_id)
            simulate(2)

            failure_detected = False
            for idx, object_id in enumerate(object_ids[:-1]): # 不检查目标物体
                position_updated_other, _ = p.getBasePositionAndOrientation(object_id)
                # if not np.allclose(position_updated[:2], initial_positions[idx][:2], atol=0.03):
                distance_other = np.linalg.norm(np.array(position_updated_other[:2]) - np.array(initial_positions_other[idx][:2]))
                if distance_other > 0.02:
                    failure_detected = True
                    break # check failure, quit
            
            if not failure_detected:
                position_updated_ego, _ = p.getBasePositionAndOrientation(target_object_id)
                # print('position_updated[:2]:{}'.format(position_updated[:2]))
                # print('position_c[:2]:{}'.format(position_c[:2]))
                distance_ego = np.linalg.norm(np.array(position_updated_ego[:2]) - np.array(position_c_ego[:2]))
                distance_ego = round(distance_ego, 4)
                print("-" * 20 + "\n" + 'moved distance: {}'.format(distance_ego))
                # if np.allclose(position_updated[:2], position_c[:2], atol=0.03):
                if distance_ego > 0.02:
                    print("target object: Failure")
                    text_id_f = p.addUserDebugText("Failure:{}".format(distance_ego), text_position, textColorRGB=text_color, textSize=text_size)
                    simulate(2)
                    p.removeUserDebugItem(text_id_f)
                else:
                    success_count += 1
                    print("target object: Success")
                    text_id_s = p.addUserDebugText("Success:{}".format(distance_ego), text_position, textColorRGB=text_color, textSize=text_size)
                    simulate(2)
                    p.removeUserDebugItem(text_id_s)
            else:
                print("-" * 20 + "\n" + "other object: Failure")
            
            # Remove all objects from the scene
            for object_id in object_ids:
                p.removeBody(object_id)
    else:
        success_count = n_trials
        
    # Save data
    print("-" * 20 + "\n" + "iteration:{} n_objects:{} success:{} success rate:{}".format(iter, n_objects, success_count, success_count / n_trials))
    data = {
        "depth_image_A": depth_image_A,
        "depth_image_B": depth_image_B,
        "success_rate": success_count / n_trials,
    }

    with open(filename, "wb") as f:
        pickle.dump(data, f)

# # stop recording
# p.stopStateLogging(logId)

# disconnect pybullet
p.disconnect()