#!/bin/bash
# $1 should be the name of the map to be copied without the extension. The map should be a pgm.

#Get to the right place
cd /home/strider/catkin_ws/src/wagon_rut_costmap/data_generation/

#fix yaml file
sed -i -e 's/png/pgm/g' maps/$1.yaml

#Make world file
cp __example.world world_files/$1.world
sed -i -e 's/__example/'$1'/g' world_files/$1.world

#Make launch files for clean map
cp clean__example.launch ../launch/clean$1.launch
sed -i -e 's/__example/'$1'/g' ../launch/clean$1.launch
cp clean__example.launch.xml ../launch/clean$1.launch.xml
sed -i -e 's/__example/'$1'/g' ../launch/clean$1.launch.xml

#Make clean map
roslaunch wagon_rut_costmap clean$1.launch map_file:="/home/strider/catkin_ws/src/wagon_rut_costmap/data_generation/maps/"$1.yaml world_file:="/home/strider/catkin_ws/src/wagon_rut_costmap/data_generation/world_files/"$1.world &
sleep 7
killall -9 roscore
sleep 2
killall -9 rviz
killall -9 stageros
killall -9 joint_state_publisher

#Make launch files for effaced map
cp effaced__example.launch ../launch/effaced$1.launch
sed -i -e 's/__example/'$1'/g' ../launch/effaced$1.launch
cp effaced__example.launch.xml ../launch/effaced$1.launch.xml
sed -i -e 's/__example/'$1'/g' ../launch/effaced$1.launch.xml

#Make effaced map
roslaunch wagon_rut_costmap effaced$1.launch map_file:="/home/strider/catkin_ws/src/wagon_rut_costmap/data_generation/maps/"$1.yaml world_file:="/home/strider/catkin_ws/src/wagon_rut_costmap/data_generation/world_files/"$1.world &
sleep 2
rosrun wagon_rut_costmap path_creator.py &
PID=$!
sleep 1
kill -SIGINT $PID
sleep 1
rosrun wagon_rut_costmap path_follower.py &
PID_follow=$!
sleep 3600 #Change to be number of seconds you want to simulate the robot driving
kill -9 $PID_follow
killall -9 roscore
sleep 2
killall -9 rviz
killall -9 stageros
killall -9 joint_state_publisher
sleep 2

#Subtract pgms
convert clean_costmaps/$1.pgm clean_costmaps/$1.pgm
convert effaced_costmaps/$1.pgm effaced_costmaps/$1.pgm
python ../scripts/subtract_pgm.py /home/strider/catkin_ws/src/wagon_rut_costmap/data_generation/effaced_costmaps/$1.pgm /home/strider/catkin_ws/src/wagon_rut_costmap/data_generation/clean_costmaps/$1.pgm /home/strider/catkin_ws/src/wagon_rut_costmap/data_generation/output/$1.pgm

#Rotate original map
convert maps/$1.pgm -flip input/$1.pgm
