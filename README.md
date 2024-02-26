# Micropolis Robotics - Robotics Software Engineer Technical Assesment

This is task defined project to asses the new hirings for Robotics Software Engineer
It helps the company to evaluate the applicants knowledge and skills in the tools and frameworks used in the department.

## Project Overview
You are given a ROS1 workspace that contains a ready to use setup for a car-like robot equibbed with a depth camera and a 3D LiDAR, placed in a virtual world within Gazebo Simulator.
The target goal for this project, is to develop a minimal user controlled autonomous navigation functionality for the robot provided.

## How To Run
- Install dependencies:
```
sudo apt-get install ros-melodic-velodyne-simulator
sudo apt-get install ros-melodic-ros-control*  ros-melodic-gazebo ros-melodic-slam-gmapping ros-melodic-move-base ros-melodic-navigation
```
- Add the models used in the world file:
~~~bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:{Add your workspace path}/src/mybot_pkg/models
~~~

- Check and install the corresponding dependencies:

~~~bash
# Available in RSE_ws
./install.sh
~~~


- Launch the robot inside Gazebo:
~~~bash
# Launch the Robot inside Gazebo World
roslaunch mybot_pkg gazebo.launch
~~~

## Milestones
To achieve the desired goal, we break it down to smaller     to be achieved in order based on its debendency for the the next step.


### 1 - Preform a SLAM on the provided world
First, we need to construct a map of the environment to use it later for offline localization. To do this we have many options and SLAM algorithms to choose from. However, we need to decide based on our environment and sensor configuration. At first, since we have a 3D LIDAR we could use LIDAR-based approaches like Slam_toolbox and Gmapping. The issue with using these methods is first we are not taking advantage of the depth camera on the robot, and since we do not have a reliable source of odometry aside from Gazebo, we will have huge map drifts and inconsistices through out the SLAM process. That is why I decided to use RTAB-MAP. In short, RTAB-Map's real-time capabilities, compatibility with 3D sensor data, effective loop closure detection, user-friendly configuration, and integration with ROS collectively makes it an ideal solution for mapping and navigation for a robot that's equipped with a depth camera and a 3D LiDAR. 
 

But before working with Rtabmap, we have to change the width of the depth image to be equal or smaller than the color image! [reference issue](https://github.com/introlab/rtabmap/issues/753). 
```
  <!-- RSE_ws/src/realsense2_description/urdf/_d435.gazebo.xacro -->
<image>
    <width>640</width>
    <height>480</height>
</image
```
We also need to generate a laserscan to use for ICP odometry to get better results in RTAB-MAP. We can do this using the `pointcloud-to-laserscan` package
```
sudo apt-get install ros-melodic-pointcloud-to-laserscan
``` 

- Now we can launch the `mybot_slam.launch` file
```
roslaunch mybot_pkg mybot_slam.launch
```
- Move the robot around manually 
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=steer_bot/ackermann_steering_controller/cmd_vel
```
- save the map (or you can just use the Rtabmap DB file)
```
rosrun map_server map_saver map:=/rtabmap/grid_map -f my_map
```

### 2 - Offline Localization
Next, to move the robot autonomously around the map we need to localize the robot in real-time without using SLAM (offline localization).</br>

To do this, offline localization is implemented using RTAB-Map. But first, let's delve into why RTAB-Map is chosen for offline localization over AMCL. Offline localization using RTAB-Map involves loading a pre-existing map generated during the mapping phase. The robot then uses this map to determine its current position without actively exploring the environment.

Choosing RTAB-Map for offline localization brings several advantages. Firstly, RTAB-Map provides a flexible and efficient way to use pre-existing maps for localization. It handles loop closure detection well, which means it can recognize and correct for errors that might occur during mapping. This capability enhances the accuracy and reliability of robot localization, especially in complex environments.

Comparatively, while AMCL is a powerful online localization algorithm, it might not be the optimal choice for scenarios where a pre-existing map is available. AMCL typically relies on the robot's motion and sensor data to estimate its pose in real-time. In offline scenarios, where the robot is not actively moving, AMCL may not perform as effectively as RTAB-Map.

Nevertheless, I have included files for localization using both AMCL and RTAB-MAP but after several tests I decided to continue with RTAB-MAP for this step.  

- launch the `mybot_slam.launch` file with localization enabled
```
roslaunch mybot_pkg mybot_slam.launch
```
- You can also do offline localization uing AMCL
```
roslaunch mybot_pkg amcl_loc.launch
```

### 3 - Autonomous Navigation with Obstacle avoidance
Once you have a represntation of the environment and you can localize your robot within it, you can then start the autonomous navigation of the robot.</br>
Implement/Use an autonomous navigation algorithm to drive the robot by itself to a defined goal that can be set in the RViz GUI, while avoiding any obstacle.
- setup move_base
```
```
- setup Navigation Stack with a car-like robot using TEB controller
```
sudo apt install ros-melodic-teb-local-planner
```

### 4 - External GUI Teleoperation
To make sure a smother operation of the robot when deploying it in the field, it's better to have a user friendly remote operation capabilties.</br>
Develop a GUI that allows use to remotly control the robot from outside a ROS environment.
Feel free to pick whatever framework or stack you like (C++ Qt, Python, Web-based GUI, etc...).
**NOTE:** Implement only the basic functionality (Drive, Steer).

### 5 - User Defined Navigation (Open)
Develop the previous milestone and adopt it so the user of your GUI can also perform the Navigation functionality (Sendg Waypoints/Goal, Mapping, etc...).

### (Optional) - Develop an Odometry Source for the robot
The very first required components to start working on any autonomous functionality are the position, orientation, velocity feedback of the robot.</br>
If we ignore the Odometry feedback provided by Gazebo, based on the robot description provided and the sensor set, develop a node that produce an Odometry feedback as accurate as possible.



```
THANK YOU!
```