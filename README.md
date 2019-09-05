
# Adaptive Monte Carlo Localization ROS
In this repo a robot uses a Hokuyo laser scanner and the Adaptive Monte Carlo Localization to localize itself inside a simulation environment. The 2D ground truth map is created from the Gazebo simulation using [`pgm_map_creator`](https://github.com/udacity/pgm_map_creator) package and the localization is performed by the `where_am_i` package by integrating the ROS AMCL package. This repo uses the [Ball-Follower-Robot repo](https://github.com/laygond/Ball-Follower-Robot) as a starting point.

[//]: # (Image References)
[image1]: ./README_images/follow.gif
[image2]: ./README_images/colormap.PNG
[image3]: ./README_images/negromap.PNG
[image4]: ./README_images/ros1.PNG

![alt text][image1]

<b>Localization:</b> Get the robot's pose, given a map of the environment.  
<b>Adaptive:</b> dynamically adjusts the number of particles over a period of time, as the robot navigates around in a map.

## Directory Structure
```
.Adaptive-Monte-Carlo-Localization
├── README.md
├── README_images						# Images used by README.md
|   └── ...
├── .gitignore							# ignores the inclusion of all map.pgm files in this repo (they are heavy)
├── my_robot
|   ├── CMakeLists.txt					# Compiler Instructions
|   ├── package.xml						# Package Info
|   ├── launch
|   |   ├── robot_description.launch	# Used by world.launch to launch robot
|   |   └── world.launch				# Launches world & robot in Gazebo and Rviz 
|   ├── meshes							# Used in my_robot.xacro for visual appearance
|   |   ├── camera.dae
|   |   └── hokuyo.dae
|   ├── urdf
|   |   ├── my_robot.gazebo				# Gazebo Plugins: Robot's sensors
|   |   └── my_robot.xacro				# Robot 
|   └── worlds
|       ├── empty.world					# Extra world
|       └── my_sexy_world.world			# Main world
|
├── pgm_map_creator
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── launch
│   │   └── request_publisher.launch	# launches nodes for map creation
│   ├── maps
│   │   └── map.pgm						# Output: 2D map generated
│   ├── msgs
│   │   ├── CMakeLists.txt
│   │   └── collision_map_request.proto
│   ├── src
│   │   ├── collision_map_creator.cc
│   │   └── request_publisher.cc
│   └── world
│       └── my_sexy_world.world			# Input: edited map (main world + map pluging)
│        
└── where_am_i
    ├── CMakeLists.txt
    ├── package.xml
    ├── config							# More Parameters nicely written for amcl.launch to use 
    │   ├── base_local_planner_params.yaml
    │   ├── costmap_common_params.yaml
    │   ├── global_costmap_params.yaml
    │   ├── local_costmap_params.yaml
    ├── launch
    │   └── amcl.launch					# launches the node to read map, apply AMCL, navigate robot
    └── maps
        ├── map.pgm						# Copy of output map from pgm_map_creator
        └── map.yaml					# map description for amcl node to apply thresholding
```

#### ROS Packages 
- `my_robot` holds the robot physical design and world environment with pluggins to interact with actuators and sensors.
- `pgm_map_creator` converts a given world into a 2D map in pgm format
- `where_am_i` uses the final output map along with its metadata to localize a robot's pose using [ROS AMCL package](http://wiki.ros.org/amcl)

#### ROS Nodes
- [ROS Teleop](http://wiki.ros.org/teleop_twist_keyboard) ROS teleop package sends command to the robot using keyboard or controller. 
`where_am_i` add nodes through its amcl.launch file:
- [map_server node] (http://wiki.ros.org/map_server) provides map data as a ROS service to other nodes such as the amcl node. Here, map_server node will locate the map you created and send it out as map data.
- [amcl node] takes odometry and laser scan data to perform the AMCL localization.
- [move_base node] uses a navigation goal position provided either through RViz or shell so that the robot will navigate to that goal position. It utilizes a costmap to determine occupied and unoccupied areas. As the robot moves around, a local costmap, in relation to the global costmap, keeps getting updated allowing the robot to move along and around the obstacle.

#### Gazebo Preexisting Plugins
In `my_robot.gazebo` you find the following plugins to interact with Rviz and Gazebo.
- `libgazebo_ros_diff_drive.so` is the plugin for the wheel joints. It accepts information from the robot's model: joint names, link dimensions, etc to calculate and publish the robot's odometry information to the topic that you specify. In our case cmd_vel topic
- `libgazebo_ros_camera.so` is the plugin for the camera sensor. It requires the camera urdf link name and it publishes to the camera topic: /camera/rgb/image_raw topic.
- `libgazebo_ros_laser.so` is the plugin for the hokuyo lidar. It requires hokuyo urdf link name and it publishes to the hokuyo topic: /scan.

## Project Overview

![alt text][image1]
- General Initial Set Up
- Generate ground truth map from Gazebo World
- Launch world environment and robot
- Apply AMCL Localazation
- Move robot around
- Select proper localization parameters

Before we start, if you are working with a native ROS installation or using a VM, some of the following package might need to be installed. You could install them from your terminal as shown below:
```sh
$ sudo apt-get install ros-kinetic-navigation
$ sudo apt-get install ros-kinetic-map-server
$ sudo apt-get install ros-kinetic-move-base
$ sudo apt-get install ros-kinetic-amcl
$ sudo apt-get install ros-kinetic-teleop-twist-keyboard
$ sudo apt-get install libignition-math2-dev protobuf-compiler
```

## General Initial Setup
#### Create a catkin_ws (unless you already have one!)
This can be done at any any directory level you want
```sh
$ mkdir -p catkin_ws/src/
$ cd catkin_ws/src/
$ catkin_init_workspace
```
#### Clone the repo (in catkin_ws/src/)
```sh
$ git clone https://github.com/laygond/Adaptive-Monte-Carlo-Localization.git
```
#### Install packages Dependencies (in catkin_ws/, therefore:)
```sh
$ cd ..
$ source devel/setup.bash
$ rosdep -i install my_robot
$ rosdep -i install pgm_map_creator
$ rosdep -i install where_am_i
```
#### Build packages (in catkin_ws/)
```sh
$ catkin_make
$ source devel/setup.bash
```
<b>Note:</b> whenever it is specified to open a new terminal do not forget to source your catkin_ws like the previous line.

## Map Creation
We need a map so that `where_am_i` package can use it for localization. Use the Directory Structure to guide yourself thoughout this section.

![alt text][image2]  ![alt text][image3]

We will make use of files from the following packages in the this order:
- my_robot
- pgm_map_creator
- where_am_i

### Steps
1. Place a copy of the Gazebo world from `my_robot` in the world folder from `pgm_map_creator`
2. In `pgm_map_creator` add the map creator plugin tag towards the end of the copied map file (just before </world> tag):
```
<plugin filename="libcollision_map_creator.so" name="collision_map_creator"/>
```
3. In `pgm_map_creator` create a map inside the map folder using the world from the world folder by doing this:
Open a terminal at your pgm_map_creator directory level and run gzerver with the edited world file:
```sh
gzserver world/<YOUR EDITED GAZEBO WORLD FILE>
```
Open another terminal anywhere inside catkin_ws, launch the request_publisher node
```sh
roslaunch pgm_map_creator request_publisher.launch
```
4. Do a quick check of the map inside map folder. If the map is cropped, you might want to adjust the parameters in `launch/request_publisher.launch`, namely the x and y values, which defines the size of the map:
```
  <arg name="xmin" default="-15" />
  <arg name="xmax" default="15" />
  <arg name="ymin" default="-15" />
  <arg name="ymax" default="15" />
  <arg name="scan_height" default="5" />
  <arg name="resolution" default="0.01" />
  ```
Remember, the map is a [pgm file](https://en.wikipedia.org/wiki/Netpbm_format), which is simply a grayscale image file, which means you could edit it using image processing softwares.

5. Place a copy of the map file from `pgm_map_creator` in the map folder from `where_am_i`

6. Finally, in the map folder from `where_am_i`, create a yaml file providing the metadata about the map.

The metadata is needed by `where_am_i` so that its AMCL can treat 'darker' pixels as obstacle in the pgm map file, and 'lighter' pixels as free space. The threshold could be set as a parameters.
In your map yaml file add the following lines:
```
image: <YOUR MAP NAME>
resolution: 0.01
origin: [-15.0, -15.0, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
```
Note: if the default map size is 30 by 30, the origin will be [-15, -15, 0]

## Launch world simulation: load robot in Gazebo and Rviz
From anywhere inside catkin_ws
```sh
$ roslaunch my_robot world.launch
```

## Localize Robot: AMCL
From anywhere inside catkin_ws
```sh
$ roslaunch where_am_i amcl.launch
```

![alt text][image4]

## Move Robot Around
You have several options to control your robot while it localize itself:
- Send navigation goal via RViz to `move_base node` 
- Send move command via teleop_twist_keyboard package.
- ... and publishing directly to the wheel actuators 

#### Publishing Directly
Open a new terminal window anywhere in the catkin_ws and publish velocity commands directly to the robot's wheel actuators. To stop vehicle publish zero values and then Ctrl + C.
```sh
$ rostopic pub /cmd_vel geometry_msgs/Twist  "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.1" 
```
