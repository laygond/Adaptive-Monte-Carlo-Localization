
# Ball Follower Robot
This repo explain the process of localizing a white ball through a robot's onboard camara and driving a differential robot torwards the ball. This repo uses [Udacity's RoboND-simple_arm repo](https://github.com/udacity/RoboND-Simple_arm) as a guide for the pub-sub architecture. 


![alt text](images/simulation.png)

### Directory Structure

#### ROS Packages 
- `my_robot` holds the robot physical design and pluggins to interact with actuators and sensors.
- `ball_chaser`holds the nodes in charge of localizing a white ball and driving the bot torwards the ball.

#### ROS Nodes
- `process_image`
- `drive_bot`

#### Gazebo Preexisting Plugins
Shared object file created from compiling C++ source code. They allow interaction with Rviz and Gazebo.
- `libgazebo_ros_diff_drive.so` accepts information from the robot's model: joint names, link dimensions, etc to calculate and publish the robot's odometry informtion to the topic that you specify.   




### Steps to Launch Simulation

#### Create a catkin_ws in your own /home/workspace directory (unless you already have one!)
/home/workspace can be any directory you want
```sh
$ cd /home/workspace/
$ mkdir -p catkin_ws/src/
$ cd catkin_ws/src/
$ catkin_init_workspace
```

#### Clone the repo in catkin_ws/src/
```sh
$ cd /home/workspace/catkin_ws/src/
$ git clone https://github.com/laygond/Ball-Follower-Robot.git
```

#### Install packages Dependencies
```sh
$ cd /home/workspace/catkin_ws
$ source devel/setup.bash
$ rosdep -i install my_robot
$ rosdep -i install process_image
```

#### Build packages
```sh
$ cd /home/workspace/catkin_ws/ 
$ catkin_make
$ source devel/setup.bash
```

### Part 1: Interact with robot
#### Launch simulation: load robot in Gazebo and Rviz
From anywhere inside catkin_ws
```sh
$ roslaunch my_robot world.launch
```

#### Move robot 
Open a new terminal and type the following:
```sh
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ rosservice call /arm_mover/safe_move "joint_1: 0.0 joint_2: 0.0"
```

## How to view image stream from the camera?
Camera image stream is published to the following topic:
```
/rgb_camera/image_raw
```

This stream can be viewed by following command in separate terminal:
```sh
$ rosrun image_view image_view image:=/rgb_camera/image_raw
```

## Simulation Interface:


