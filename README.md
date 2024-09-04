# MultiCam PointCloud (MCPC)

[![ROS2Humble](https://img.shields.io/badge/ROS2_Humble-Ubuntu_22.04-blue.svg)](https://docs.ros.org/en/humble/index.html)
[![ROS2Jazzy](https://img.shields.io/badge/ROS2_Jazzy-Ubuntu_24.04-green.svg)](https://docs.python.org/3/whatsnew/3.10.html)

This repository represents our implementation for a multicam imaging system for the [ROS2 Farmbot implementation](https://github.com/PetriJF/FarmBot_ROS2).
It was tested on both ROS2 Humble and Jazzy, and it includes additional launch files and run files to run over the standard ones from the [ROS2 Farmbot implementation](https://github.com/PetriJF/FarmBot_ROS2)

We recommend getting a USB HAT for the Raspberry Pi that can be powered externally as a RPi might not have enough current to power multiple cameras. For our implementation we use the Luxonis OAK-D Lite (Non FF version). Before running the experiment, you should map the depth focus lengths, as they vary slightly from camera to camera (see [Camera Config](config/luxonis_oak_d_lite_camera_config.yaml)).

Finally, our implementation work for two use cases:
- Potted plants (System will move from pot to pot and image both sides of the plant)
- Row plants (System will travel and image along the row and then move to the other side and do the same)

Both of these implementations are assuming you are using our MCPC 3D printed rig, that enables to take images at different points in space around the plant.

# How to run

First, run the alternative launch file (runs all the normal nodes, except for the standard camera one), in addition to the luxonis imaging node. We recommend not saving the images on the RPi's SDCard, as it can fill up fast if you are taking 4K images. In our experiments we are sending them to a NAS, but other methods can be also used (e.g. if you are using the RPi5, you can attach an SSD and save them locally).
``` bash
ros2 launch multicam_pointcloud standard_multicam.launch.py image_save_path:="YOUR_IMAGE_FOLDER"
```

Run the MCPC Controller. This controller is used to send all the commands to the different nodes that are launched. It includes some standard commands (e.g. e, H_0 and C_0) but it also contains the sequence generation code for the imaging (Note that the plants need to be mapped in the map_handler in order for the MCPC Controller to be able to generate a sequence).
``` bash
ros2 run multicam_pointcloud mcpc_node
```

Example for sequence generation:
``` bash
FORM_TYPE GRID 10 6 0 0 375 

# FORM_TYPE PATTERN COLS ROWS MEASURE_DIST TURN_SIDE D_LEN
# MEASURE_DIST = 0 -- for pots
# TURN_SIDE = 0 for turn at Y-0, 1 for turn at Y-max
# D_LEN = Top camera offset (configurable)
```