# URDF_models

### This repository contains a collection of over 100 URDF models for use with Pybullet. 

### To facilitate the use of this database, I provide some extra information as shown in the Excel table below. 

| index | subfolder_name   | relative_path                | size_x     | size_y     | size_z     | object_name     | euler_x | euler_y | euler_z | object_description                                                                                           |
|-------|------------------|------------------------------|------------|------------|------------|-----------------|---------|---------|---------|-------------------------------------------------------------------------------------------------------------|
| 0     | black_marker     | black_marker/model.urdf      | 0.023168   | 0.151316   | 0.022861   | black_marker    | 0       | 0       | 0       | a black marker pen                                                                                          |
| 1     | bleach_cleanser  | bleach_cleanser/model.urdf   | 0.109653   | 0.07443    | 0.257586   | bleach_cleanser | 0       | 0       | 0       | a white bleach cleanser with the "Soft Scrub" brand label on it. The label is blue and green.                |
| 2     | blue_cup         | blue_cup/model.urdf          | 0.087695   | 0.112706   | 0.074358   | blue_cup        | 0       | 0       | 0       | A greyish-blue mug.                                                                                         |
| 3     | blue_marker      | blue_marker/model.urdf       | 0.023173   | 0.15136    | 0.022867   | blue_marker     | 0       | 0       | 0       | A marker with a blue cap and a black body.                                                                  |
| 4     | blue_moon        | blue_moon/model.urdf         | 0.063152   | 0.102187   | 0.258420   | blue_moon       | 0       | 0       | 0       | Laundry detergent in a red bottle with a Chinese brand label reading "Blue Moon". The bottle cap is darker. |
| ...     | ...        | ...         | ...   | ...   | ...   | ...       | ...       | ...       | ...       | ... |


- **subfolder_name**: name of the folder where each URDF model is stored. 

- **relative_path**: path of each URDF model

- **size_x, size_y, size_z**: dimensions of each URDF model 

- **object_name**: name of each URDF model, which is the same as **subfolder_name**. 

- **euler_x, euler_y, euler_z**: recommended Euler angles for loading the items, which can prevent unreasonable scenes due to improper angles of the items, such as a chair being tilted. 

- **object_description**: a human-annotated description of each URDF model 

### Note that Some of the models are sourced from the [pybullet-URDF-models](https://github.com/ChenEating716/pybullet-URDF-models) project

- we would like to extend our gratitude to the author for their hard work. 

- This repository is constantly updated and improved.💪💪💪

### ✨✨✨ **An interesting tool**: [display objects](https://colab.research.google.com/drive/1qLF2JoN9AXtYcFIgmnK8p0TFTuGG0tEB?usp=sharing)
