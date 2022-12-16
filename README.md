# Hybrid Local Planner
This is an implementation of ROS Local Planner using the Dijkstra's algorithm and the Hybrid A star algorithm

## Experiments
I have conducted extensive experiments to compare the performance of the Hybrid Local Planner with DWA Planner and TEB Planner. DWA Planner and TEB Planner are the two most commonly used local planners in ROS. All my experiments are recorded and the recording can be found here: https://drive.google.com/file/d/1Ai8MCKzv-_lAKkK3HQZqoNwSQ7tNDZRI  
The contents of the video recording are as follows: 
Experiment Name | Time in video |
--- | --- |
DWA Planner in static obstacle environment (Run 1 to 5) | 00:20 to 10:24 |
Hybrid Planner in static obstacle environment (Run 1 to 5) | 10:24 to 15:50 |
TEB Planner in static obstacle environment (Run 1 to 2) | 15:50 to 17:32 |
DWA Planner in dynamic obstacle environment<br>Obstacle velocity: 0.4, 0.5, 0.6, and 0.7 m/s | 17:32 to 20:25|
Hybrid Planner in dynamic obstacle environment<br>Obstacle velocity: 0.5, 0.6, 0.7, 0.8, and 0.9 m/s|20:25 to 22:37|
TEB Planner in dynamic obstacle environment<br>Obstacle velocity: 0.3, 0.4 and 0.5 m/s | 22:37 to 24:16 |


## Installation
Before starting, please make sure that the GPU drivers are up to date. It is recommended to use Nvidia GPUs with Gazebo. 

The first thing to do is to setup the TurtleBot3 ROS packages. The installation guide can be found here: https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup

There are **different ROS versions** for **different Ubuntu versions**. In my setup, I used Ubuntu 20.04.5 LTS and ROS1, so my selected ROS package was **noetic**. 
I am providing the necessary commands for installing ROS1 noetic here:
```
$ sudo apt update
$ sudo apt upgrade
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic.sh
$ chmod 755 ./install_ros_noetic.sh 
$ bash ./install_ros_noetic.sh

$ sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
  
$ sudo apt install ros-noetic-dynamixel-sdk
$ sudo apt install ros-noetic-turtlebot3-msgs
$ sudo apt install ros-noetic-turtlebot3
```

Next, add the line `export TURTLEBOT3_MODEL=burger` at the end of the `~/.bashrc` file.

After that, TurtleBot3 Gazebo Simulator needs to be installed.   
  
Before installing Gazebo, copy the [model_editor_models](https://github.com/fahimfss/Hybrid_Local_Planner/tree/main/model_editor_models) **folder** with it's contents to the **Home** directory (~/model_editor_models/)  

Gazebo works best with an Nvidia GPU, and I used an RTX 3070 in my setup. Please make sure the GPU drivers are up to date before installing Gazebo. The instructions for installing Gazebo can be found here:
https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation  
**Note: Restart the terminal before installing Gazebo**

I am providing the code for installing TurtleBot3 Gazebo Simulator for ROS1 noetic here.
```
$ cd ~/catkin_ws/src/
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/catkin_ws && catkin_make
```
Restart the terminal and verify Gazebo is installed correctly using the following command:
```
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
Use `ctrl + c` in the terminal to close the Gazebo simulator.  
  
After that, multiple files are required to be placed in the correct folders to run the experiments correctly.
* Copy the **contents** of the [Maps](https://github.com/fahimfss/Hybrid_Local_Planner/tree/main/Maps) **folder** to the **Home** directory (~/map_dynamic_obs.pgm, ~/map_dynamic_obs.yaml, ~/map_static_obs.pgm, ~/map_static_obs.yaml)
* Copy the two **contents** of the [Worlds](https://github.com/fahimfss/Hybrid_Local_Planner/tree/main/Worlds) **folder** inside the `~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds` directory 
* Copy the two **contents** of the [Launch](https://github.com/fahimfss/Hybrid_Local_Planner/tree/main/Launch) **folder** inside the `~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch` directory 
* Lastly, copy the [navigation](https://github.com/fahimfss/Hybrid_Local_Planner/tree/main/navigation) folder to the `~/catkin_ws/src` directory (~/catkin_ws/src/navigation)

Use the command `$ cd ~/catkin_ws && catkin_make` to compile the code.

## Running the Experiments
To run the experiments, first, we need to set the Local Planner in the [move_base.launch](https://github.com/fahimfss/Hybrid_Local_Planner/blob/main/move_base.launch) file. In my pc, the move_base.launch file is located at `/opt/ros/noetic/share/turtlebot3_navigation/launch/move_base.launch`.  
  
To edit the file, use the following command: `$ sudo nano /opt/ros/noetic/share/turtlebot3_navigation/launch/move_base.launch`  
  
By default, the DWA local planner is used.  
To change to Hybrid Local Planner, use: `<param name="base_local_planner" value="hybrid_local_planner/HybridPlannerROS"/>` inside `<node>`  
To change to TEB Local Planner, use: `<param name="base_local_planner" value="teb_local_planner/TebLocalPlannerROS"/>` inside `<node>`. (TEB local planner can be installed using `$ sudo apt-get install ros-noetic-teb-local-planner`)   
  
Please comment out other local planners which are not intended for use. You can check the [move_base.launch](https://github.com/fahimfss/Hybrid_Local_Planner/blob/main/move_base.launch) file for example. Please save the move_base.launch file after making any updates.  
  
### Running the Static Obstacle Experiment
In a new terminal, run the following command to run Gazebo static obstacle environment:  
`$ roslaunch turtlebot3_gazebo turtlebot3_rec_static_obs.launch`

After the Gazebo simulator fully starts, in another terminal, run the following command to run RViz:  
`$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map_static_obs.yaml`  
  
In RViz, I unselect the **Amcl Particles** and **Local Map > Costmap** for better observability. To see the Hybrid Local Planner's selected path, 
select `Add > By topic > /move_base > HybridPlannerROS > /visualization_marker > Marker` and click `OK`.  
  
Press the **middle mouse button** and move the mouse to move the map in RViz. Click the `2D Nav Goal` button, click and drag on the map using the mouse to set a goal and direction for the robot. 

![RViz](https://user-images.githubusercontent.com/8725869/207963984-7be73377-06d3-4497-abc4-448b8dda11a9.png)
  
The robot should start moving towards the goal.

**Note: Hybrid Local Planner can only handle forward movement at this point. It might crash if the global planner provides backward paths. In that case, please restart both Gazebo and RViz (use `ctrl + c` to close Gazebo and RViz first)**



### Running the Dynamic Obstacle Experiment
Running the dynamic obstacle experiment is slightly tricky, as you need to keep track of time. The obstacle starts to move after 20 seconds at the speed of 0.5 m/s. Both of these values can be changed in the `~/catkin_ws/src/navigation/MoveObject/src/model_push.cc` file (lines 35, 36). Please use the command `cd ~/catkin_ws && catkin_make` to compile the code if the values are changed in the `model_push.cc` file.

In a new terminal, run the following command to run the Gazebo dynamic obstacle environment (After the Gazebo simulator starts, keep track of time to decide when to set the goal command):  
`$ roslaunch turtlebot3_gazebo turtlebot3_rec_dynamic_obs.launch`

After the Gazebo simulator fully starts, in another terminal, run the following command to run RViz:  
`$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map_dynamic_obs.yaml`  
  
Two seconds before the obstacle starts moving (in this case, at 18-second mark after launching the Gazebo simulator), set the goal using the `2D Nav Goal` button. 

## File Descriptions
File Descriptions
This section describes the files in [navigation/hybrid_local_planner/src](https://github.com/fahimfss/Hybrid_Local_Planner/tree/main/navigation/hybrid_local_planner/src). The corresponding header files are provided in [navigation/hybrid_local_planner/include](https://github.com/fahimfss/Hybrid_Local_Planner/tree/main/navigation/hybrid_local_planner/include).

[hybrid_planner_ros.cpp](https://github.com/fahimfss/Hybrid_Local_Planner/blob/main/navigation/hybrid_local_planner/src/hybrid_planner_ros.cpp): This file acts as the primary set-up code for hybrid local planner. It calls the hybrid planner for calculating and providing the velocity the robot should follow, and other useful methods also. This file follows the implementation of [DWAPlannerROS](http://docs.ros.org/en/lunar/api/dwa_local_planner/html/dwa__planner__ros_8cpp_source.html).  
  
[hybrid_planner.cpp](https://github.com/fahimfss/Hybrid_Local_Planner/blob/main/navigation/hybrid_local_planner/src/hybrid_planner.cpp): This file contains the Hybrid Local Planner algorithm. The function `findBestPath` (line 593), takes input the current state of the robot and calculates and returns the best trajectory to follow and the corresponding velocities (both Translational and Rotational). Hybrid Local Planner works by first creating a graph in close proximity in front of the robot. Then, Dijkstra's algorithm is used to find the best path in this graph. Using the path, the allowed velocity is calculated based on closeness to obstacles. Based on the allowed velocity, a point in the path is selected as the goal point. After that, the best heading position for the goal point is selected. Now we have a goal point and a heading direction. The Hybrid A Star algorithm is used next to find a path from the current position and heading to reach the goal position and heading.  
  
[Node.cpp](https://github.com/fahimfss/Hybrid_Local_Planner/blob/main/navigation/hybrid_local_planner/src/Node.cpp): The Node class represents a node in the Dijkstra's algorithm.  

[Helper.cpp](https://github.com/fahimfss/Hybrid_Local_Planner/blob/main/navigation/hybrid_local_planner/src/Helper.cpp): The Helper class contains useful static methods that Hybrid Planner uses throughout the implementation.   
  
[Edge.cpp](https://github.com/fahimfss/Hybrid_Local_Planner/blob/main/navigation/hybrid_local_planner/src/Edge.cpp): The Edge class represents an Edge in the Dijkstra's algorithm.   
  
[AStar_Node.cpp](https://github.com/fahimfss/Hybrid_Local_Planner/blob/main/navigation/hybrid_local_planner/src/AStar_Node.cpp): The AStar_Node class represents a node in the Hybrid A Star algorithm.  


