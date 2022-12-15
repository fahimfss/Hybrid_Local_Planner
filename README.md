# Hybrid Local Planner
This is an implementation of ROS Local Planner using the Dijkstra's algorithm and the Hybrid A star algorithm

## Installation
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

Next, add the line `export TURTLEBOT3_MODEL=burger` to `~/.bashrc`.

After that, TurtleBot3 Gazebo Simulator needs to be installed. Gazebo works best with a Nvidia GPU and I used a RTX 3070 in my setup. Please make sure the gpu drivers are up to date before installing Gazebo. The instructions can be found here:
https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation  
**Note: Restart the terminal before installing Gazebo**

I am providing the code for installing TurtleBot3 Gazebo Simulator for ROS1 noetic here.
```
$ cd ~/catkin_ws/src/
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/catkin_ws && catkin_make
```
Restart the terminal and verify Gazebo is installed correctly using the following comand:
```
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
Use `ctrl + c` in the terminal to close Gazebo simulator.  
  
After that, multiple files are required to be placed in correct folders to run the experiments correctly.
* Copy the [model_editor_models](https://github.com/fahimfss/Hybrid_Local_Planner/tree/main/model_editor_models) **folder** to the **Home** directory (~/model_editor_models/).
* Copy the **contents** of the [Maps](https://github.com/fahimfss/Hybrid_Local_Planner/tree/main/Maps) **folder** to the **Home** directory (~/map_dynamic_obs.pgm, ~/map_dynamic_obs.yaml, ~/map_static_obs.pgm, ~/map_static_obs.yaml)
* Copy the two **contents** of the [Worlds](https://github.com/fahimfss/Hybrid_Local_Planner/tree/main/Worlds) **folder** inside the `~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds` directory 
* Copy the two **contents** of the [Launch](https://github.com/fahimfss/Hybrid_Local_Planner/tree/main/Launch) **folder** inside the `~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch` directory 
* Lastly, copy the [navigation](https://github.com/fahimfss/Hybrid_Local_Planner/tree/main/navigation) folder to the `~/catkin_ws/src` directory (~/catkin_ws/src/navigation)

Lastly, use the command `cd ~/catkin_ws && catkin_make` to install the Hybrid Local Planner package.

## Running the Experiments
To run the experiments, first we need to set the Local Planner in the [move_base.launch](https://github.com/fahimfss/Hybrid_Local_Planner/blob/main/move_base.launch) file. In my pc, the move_base.launch file is located at `/opt/ros/noetic/share/turtlebot3_navigation/launch/move_base.launch`.  
  
To edit the file, use the following command: `$ sudo nano /opt/ros/noetic/share/turtlebot3_navigation/launch/move_base.launch`  
  
By default the DWA planner is used.  
To change to Hybrid Local Planner, use: `<param name="base_local_planner" value="hybrid_local_planner/HybridPlannerROS"/>` inside `<node>`  
To change to TEB Local Planner, use: `<param name="base_local_planner" value="teb_local_planner/TebLocalPlannerROS"/>` inside `<node>`. (TEB local planner can be installed using `$ sudo apt-get install ros-noetic-teb-local-planner`)   
  
Please comment out other local planners which are not intended for use. You can check the [move_base.launch](https://github.com/fahimfss/Hybrid_Local_Planner/blob/main/move_base.launch) file for example. Please save the move_base.launch file after making any updates.  
  
### Running the Static Obstacle Experiment
In a new terminal, run the following command to run Gazebo static obstacle environment:  
`$ roslaunch turtlebot3_gazebo turtlebot3_rec_static_obs.launch`

In another terminal, run the following command to run RViz:  
`$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map_static_obs.yaml`  
  
In RViz, I unselect the **Amcl Particles** and **Local Map > Costmap** for better observability. To see the Hybrid Local Planner's selected path, 
select `Add > By topic > /move_base > HybridPlannerROS > /visualization_marker > Marker` and click `OK`.  
  
Press the **middle mouse button** and move the mouse to move the map in RViz. Click the `2D Nav Goal` button, click and drag on the map using the mouse to set a goal and direction for the robot. 

![RViz](https://user-images.githubusercontent.com/8725869/207963984-7be73377-06d3-4497-abc4-448b8dda11a9.png)
  
The robot should start moving towards the goal.

**Note: Hybrid Local Planner can only handle forward movement at this point. It might crash if the global planner provides backward paths. In that case, please restart both Gazebo and RViz (use `ctrl + c` to close Gazebo and RViz first)**













