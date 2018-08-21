#!/bin/bash
# $1 should be the name of the map to be copied without the extension. The map should be a pgm.

#Make world file
cp __example.world world_files/$1.world
sed -i -e 's/__example/'$1'/g' world_files/$1.world

#Make launch files for clean map
cp clean__example.launch ../launch/clean$1.launch
sed -i -e 's/__example/'$1'/g' ../launch/clean$1.launch
cp clean__example.launch.xml ../launch/clean$1.launch.xml
sed -i -e 's/__example/'$1'/g' ../launch/clean$1.launch.xml

#Make clean map
roslaunch wagon_rut_costmap $1.launch map_file:="/home/strider/catkin_ws/src/wagon_rut_costmap/data_generation/maps/"$1.yaml world_file:="/home/strider/catkin_ws/src/wagon_rut_costmap/data_generation/world_files/"$1.world &
# pids= pgrep -f /opt/ros/kinetic/lib/rosout/rosout
# echo $pids
sleep 10
# kill -9 $PID
# killall roslaunch
# kill $(pgrep command)
# kill -s KILL -- -gpid
# killall -9 "teleop-14"
# kill -9 $pids
# pgrep -f /opt/ros/kinetic/lib/rosout/rosout
# xargs kill -9
# killall -9 /opt/ros/kinetic/lib/rosout/rosout
killall -9 roscore
sleep 2
# killall -9 rosmaster
sleep 2
echo "GOT TO 32"

#Make launch files for effaced map
cp effaced__example.launch ../launch/effaced$1.launch
sed -i -e 's/__example/'$1'/g' ../launch/effaced$1.launch
cp effaced__example.launch.xml ../launch/effaced$1.launch.xml
sed -i -e 's/__example/'$1'/g' ../launch/effaced$1.launch.xml

#Make effaced map
roslaunch wagon_rut_costmap effaced_$1.launch map_file:="/home/strider/catkin_ws/src/wagon_rut_costmap/data_generation/maps/"$1.yaml world_file:="/home/strider/catkin_ws/src/wagon_rut_costmap/data_generation/world_files/"$1.world &
sleep 5
rosrun wagon_rut_costmap path_creator.py &
PID=$!
sleep 5
kill -9 $PID
sleep 2
rosrun wagon_rut_costmap path_follower.py &
PID_follow=$!
sleep 5 #Change to be way longer
kill -9 $PID_follow
killall -9 roscore
sleep 2

#Next need to subtract the two maps
