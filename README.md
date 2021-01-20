https://github.com/ElijahAlmanzor/Robot_Programming

# 1. Chosen Focus Area:

The specialisation of this work is *perception* in a *static* environment using the *realistic* crop rows with a single **Thorvald** robot.

# 2. Summary of the Solution:

The perception problem is tackled using a [ROS YOLO implementation](https://github.com/leggedrobotics/darknet_ros) of the [Darknet CNN](https://github.com/AlexeyAB/darknet). YOLO was trained on 676 [manually labelled](https://github.com/AlexeyAB/Yolo_mark) images taken from simulations for localisation and detection of the crops and weeds.

Thorvald is moved to the start of the three pairs of crop rows using **Move_base** to position it with high accuracy. To traverse the crop rows however, a simple proportional controller is used to move Thorvald in a straight direction towards the end of the crop rows. This allows for periodical stopping such that any weeds close to the center of the camera is sprayed with herbicide.

# 3. Instructions:

## Running the Solution:
1. Ensure that [YOLO ROS](https://github.com/leggedrobotics/darknet_ros) package is installed in your workspace - this is responsible for the CNN object detection.
1. Ensure that the [uol_cmp9767m_tutorial](https://github.com/LCAS/CMP9767M/tree/master/uol_cmp9767m_tutorial) package is also intalled in your workspace.
1. Ensure that CUDA is installed for your NVIDIA GPU.
1. Overwrite the above packages using the data files from this repository. 
1. Ensure python libraries such as opencv, numpy, cvbridge and actionlib are installed.

Also run the command:
``` 
    sudo apt-get install \
    ros-melodic-robot-localization \
    ros-melodic-topological-navigation \
    ros-melodic-amcl \
    ros-melodic-fake-localization \
    ros-melodic-carrot-planner
    ros-melodic-opencv-apps \
    ros-melodic-rqt-image-view \
    ros-melodic-uol-cmp9767m-base \
    ros-melodic-find-object-2d \
    ros-melodic-video-stream-opencv \
    ros-melodic-topic-tools \
    ros-melodic-rqt-tf-tree
``` 
5. Download the trained weights and custom configuration files (transfer learnt from the base YOLOv3 weights trained on the COCO dataset) to allow for prediction of the 3 variants of the crops and the weeds.
https://drive.google.com/drive/folders/1PWvg936u5Dtwgpnn0b8kiIc85OiSia5k?usp=sharing


Place the weights in the directory:
> /darknet_ros/yolo_network_config/weights/yolo-obj.weights


and the configuration file in the directory:
> /darknet_ros/yolo_network_config/cfg/yolo-obj.cfg



6. Replace the sensors.xacro in the uol_cmp9767_base/urdf folder with the one provided in this repository to obtain the Thorvald model with the spray nozzle positioned to the side of the camera angled to face the centre of the kinect camera's focal point.


7. Ensuring to *catkin_make* and *source* the directory first.
Then run:
```bash 
roslaunch uol_cmp9767m_tutorial yolo_thorvald.launch 
```
to run the simulation of Thorvald spraying the weeds on the three pairs of crop rows with varying difficulty.
The 4 different plants have been classified with labels cropa, cropb, cropc and weed.

## Critical Python Components
1. subscribe_bbox.py (bbox_display node) is responsible for subscribing to the ROS YOLO predictions.
2. move_and_spray_weeds.py (move_base_sequence node) is responsible for moving the Thorvald as well as spraying.
3. sprayer.py (sprayer node) is responsible for applying the spray blobs at the centre of the camera focal point.
4. take_dataset.py (image_converter node) was used to gather the dataset.


## Step-by-Step Reproduction of the Solution/ Train from Scratch:
1. The first step is to gather the dataset on which the pre-trained YOLOv3 weights will be trained on. Use the *take_dataset.py* python file to take images from the Kinect camera.
Ensure to create a ./dataset directory in the folder of the script as that is where the taken images will go. 

Use the command:
```bash 
rosrun key_teleop key_teleop.py key_vel:=/thorvald_001/teleop_joy/cmd_ve
```
to move Thorvald manually whilst taking photos. 

2. Follow the instructions on https://github.com/AlexeyAB/darknet#how-to-train-to-detect-your-custom-objects to figure out how to create the configuration files needed for the training of the Yolo/Darket and where to download the pre-trained YOLOv3 weights.

3. Once all images have been taken, label using the [Yolo_mark](https://github.com/AlexeyAB/Yolo_mark) program. Follow their instructions on how to use. 
Must make a separate directory for this. 
Use:
```bash
source ./linux_mark.sh
```
 to run the program. 

4. Follow and modify the tutorial provided in https://medium.com/@quangnhatnguyenle/how-to-train-yolov3-on-google-colab-to-detect-custom-objects-e-g-gun-detection-d3a1ee43eda1 to be able to **train on GoogleCollabs Tesla GPU** with the desired crop and weed classification and detection. Takes approximately a 12 hours to train on 12000 iterations.

5. Install CUDA on your computer to ensure smooth real-time object detection. [ROS Yolo](https://github.com/leggedrobotics/darknet_ros) automatically detects if CUDA is installed or not. Edit the CMakeLists.txt file in *darknet_ros* to add the GPU achitecture/compute capability of your [GPU](https://en.wikipedia.org/wiki/CUDA#Supported_GPUs).
For example:
> -O3 -gencode arch=compute_62,code=sm_62

6. Place the trained weight and modified cfg files in the */yolo_network_config* directory.
7. Edit the *ros.yaml* from the *darknet_ros/config* directory to change the image topic YOLO is subscribing to:
```
  camera_reading:
    topic: /thorvald_001/kinect2_camera/hd/image_color_rect
    queue_size: 1
```
Also create a new file called *yolo-obj.yaml* and paste this as the content:
``` yolo_model:

  config_file:
    name: yolo-obj.cfg
  weight_file:
    name: yolo-obj_final.weights
  threshold:
    #Increase the threshold value so only detected objects with high probability are counted
    value: 0.5
  detection_classes:
    names:
      - cropa
      - cropb
      - weed
      - cropc
```
8. Create a copy of the *darknet_ros.launch* file and modify lines 13 and 14 to:
```  <!-- ROS and network parameter files -->
  <arg name="ros_param_file" default="$(find darknet_ros)/config/ros.yaml"/>
  <arg name="network_param_file" default="$(find darknet_ros)/config/yolo-obj.yaml"/>
```
9. As the focus of project was on perception, a simple spraying mechanism was used. The spray nozzle is moved to the side of the camera (-0.2m) and angled to face the centre of the cameras focal point (68.2 degrees due to being 0.5m above the ground) to allow for spraying when there are weeds at the center of the camera.  
Modify *sensors.xacro* in the uol_cmp9767m_base to move the sprayer at the same location as the camera. 
Under the Fake Sprayer origin, change to:


>origin xyz="0.45 -0.2 0.5" rpy="-1.2 0 0"/

10. Modify sprayer.py line 66 to:
>  request.initial_pose.position.x = 0.45

11. Now to the run the program with the crop and weed detection (making sure each terminal is sourced in the right workplace):
    1. ```roslaunch uol_cmp9767m_base thorvald-sim.launch map_server:=true```
        * This is responsible for launching the main Gazebo simulation as well as MoveBase
    2. ```roslaunch uol_cmp9767m_tutorial move_base.launch```
        * This is for moving using MoveBase with actionlib
    3. ```roslaunch darknet_ros_thor.launch```
        * This is to launch the custom YOLO object detector
    4. ```rosrun darknet_ros subscribe_bbox.py```
        * ROS node that subscribes to the bounding boxes produced by the YOLO CNN. This also displays it in RViz
    5. ```rosrun uol_cmp9767m_tutorial sprayer.py```
        * Responsible for spraying the spray blobs at the centre of the camera
    6. ```rosrun uol_cmp9767m_tutorial move_and_spray_weeds.py```
        * Node for moving Thorvald to the start of the crop rows as well as for traversing them ad spraying.
    7. Run **RViz** with the yolo_with_navbase.rviz config
        * To view the camera display with the bounding boxes in RViz

**All of these are launched in the yolo_thorvald.launch file!**
