#!/bin/bash
# $1 should be the name of the map to be copied without the extension. The map should be a pgm.

cp __example.world world_files/$1.world
sed -i -e 's/__example/'$1'/g' world_files/$1.world
roslaunch wagon_rut_costmap turtlebot_costmap_nav.launch map_file:="/home/strider/catkin_ws/src/wagon_rut_costmap/data_generation/maps/"$1.yaml world_file:="/home/strider/catkin_ws/src/wagon_rut_costmap/data_generation/world_files/"$1.world


#MAKE A BRANCH FOR THIS STUFF CUZ YOU'RE GONNA MESS UP YOUR COSTMAPS
