# wagon_rut_costmap

This repository implements a "wagon rut" layer for the ROS navigation stack costmap. This means that the cost in each location where the robot drives decreases a small amount, ultimately creating a "wagon rut" effect on the costmap, which will encourage the robot to retrace its own path in future path planning. The ultimate goal of this layer is to allow robots to navigate human spaces more naturally, by matching their autonomous path planning to paths humans have previously driven them on. For more details see the Wiki. 



Use:

In order to use this costmap layer, clone this repository into a catkin workspace. It depends on the packages catkin, costmap_2d, dynamic_reconfigure, pluginlib, roscpp, and stage_ros.

To see how it works, launch the turtlebot_costmap_nav.launch file. Make sure that you change the file paths in both src/efface_current_pose.cpp and src/load_old_map.cpp to the locations of map_a.pgm, map_b.pgm, and filename.txt in the maps folder. After doing this, the above launch file will allow you to try out the wagon rut layer on the simple Stage maze with some previous wagon ruts already in place.

In order to use this costmap on your own map, create map and world files for your map and then add them as parameters at the end of your roslaunch command, or within the launch files itself. More details on this process can be found in the following Stage tutorial:
http://wiki.ros.org/turtlebot_stage/Tutorials/indigo/Customizing%20the%20Stage%20Simulator



Repository Organization:

This repository contains the setup and code for three ROS costmap layers:
Compression Layer: Compresses the range of possible costs on the costmap from 0—255 to 50—255.
Efface Layer: Decreases the cost wherever the robot drives.
Load Old Layer: Loads in the previous costmap.

Each of these layers depends on the previous one. The effacement layer only works if the compression layer is also running, because only non-zero values can be decreased. The "load old" layer only works if the efface layer has been run at least once before, because it loads in costs saved by the efface layer. It itself is necessary in order to continue making progress on wearing out the wagon ruts instead of starting fresh with each launch.

Data generation:

The data generation branch includes a folder with all the necessary files to generate simulation data on a folder of map pgms and yaml files.
