#!/usr/bin/env python
import rospy
import smach
import copy
from smach import StateMachine
from smach_ros import SimpleActionState
import smach_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import csv
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import rosnode
import actionlib
import time
import numpy as np
import datetime
from sensor_msgs.msg import LaserScan

waypoint_list = []

#Define class for state machine, pass in smach.State
class patrol_state(smach.State):
	#Init
	def __init__(self):
		#Subscriber for keypresses
		self.keypress_sub = rospy.Subscriber('/key_monitor',String,self.key_callback)

		#Define patrol state, 4 possible outcomes (defined in main), 2 inputs and 1 output(defined in main, names have been remapped below in main),
		smach.State.__init__(self,outcomes=['outcome1','outcome2','outcome3','outcome4'],input_keys=['patrol_waypoints_in','patrol_waypoint_num_in'],output_keys=['patrol_waypoint_num_out'])
		#Boolian values to change state given keyboard callback
		self.h_bool = False
		self.r_bool = False

		#Open file to write sensor reading too (CSV)
		self.outputFile = open('sensor_readings.csv', 'w')
		self.outputWriter = csv.writer(self.outputFile)
		self.outputWriter.writerow(['Reading Number','Minimum Distance','Time Stamp'])
		self.sensor_reading_number = 0
		self.min_distance = 0

		#Scan Subscriber used for saving data at waypoints
		self.scan_sub = rospy.Subscriber('/scan',LaserScan,self.scan_callback)

	#Main funtion for patrol state
	def execute(self,userdata):
		#Bool to check/set keyboard callbacks
		if self.r_bool == True:
			self.r_bool = False
			return 'outcome2'
		if self.h_bool == True:
			self.h_bool = False
			return 'outcome1'

		#Create publisher and maker for pubishers
		waypoint_viz_pub = rospy.Publisher('waypoint_marker', Marker, queue_size=10)
		rospy.sleep(1)
		waypoint_marker = Marker()
		#Loop through waypoints creating and publishing markers
		for w in userdata.patrol_waypoints_in:
			waypoint_marker.header.frame_id = w[1]
			waypoint_marker.header.stamp = rospy.get_rostime()
			waypoint_marker.ns = "robot"
			waypoint_marker.id = float(w[0])
			waypoint_marker.type = 9
			waypoint_marker.action = 0
			waypoint_marker.pose.position.x = float(w[2])
			waypoint_marker.pose.position.y = float(w[3])
			waypoint_marker.pose.position.z = float(w[4])
			waypoint_marker.pose.orientation.x = float(w[5])
			waypoint_marker.pose.orientation.y = float(w[6])
			waypoint_marker.pose.orientation.z = float(w[7])
			waypoint_marker.pose.orientation.w = float(w[8])
			waypoint_marker.scale.x = .25
			waypoint_marker.scale.y = .25
			waypoint_marker.scale.z = .25
			waypoint_marker.color.r = 1.0
			waypoint_marker.color.g = 0.0
			waypoint_marker.color.b = 0.0
			waypoint_marker.color.a = 1.0
			waypoint_marker.lifetime = rospy.Duration(0)
			waypoint_marker.text = w[0]
			waypoint_viz_pub.publish(waypoint_marker)
		# print "Marker Placed"

		# print len(userdata.patrol_waypoints_in)
		# print "start of execute patrol state"
		self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		# print "action client started"
		self.client.wait_for_server()
		# print "got server"
		rospy.loginfo('got server')
		# print userdata.patrol_waypoints_in
		goal_pose = MoveBaseGoal()
		goal_pose.target_pose.header.stamp = rospy.Time.now()
		goal_pose.target_pose.header.frame_id = userdata.patrol_waypoints_in[userdata.patrol_waypoint_num_in][1]
		goal_pose.target_pose.pose.position.x = float(userdata.patrol_waypoints_in[userdata.patrol_waypoint_num_in][2])
		goal_pose.target_pose.pose.position.y = float(userdata.patrol_waypoints_in[userdata.patrol_waypoint_num_in][3])
		goal_pose.target_pose.pose.position.z = float(userdata.patrol_waypoints_in[userdata.patrol_waypoint_num_in][4])
		goal_pose.target_pose.pose.orientation.x = float(userdata.patrol_waypoints_in[userdata.patrol_waypoint_num_in][5])
		goal_pose.target_pose.pose.orientation.y = float(userdata.patrol_waypoints_in[userdata.patrol_waypoint_num_in][6])
		goal_pose.target_pose.pose.orientation.z = float(userdata.patrol_waypoints_in[userdata.patrol_waypoint_num_in][7])
		goal_pose.target_pose.pose.orientation.w = float(userdata.patrol_waypoints_in[userdata.patrol_waypoint_num_in][8])
		self.client.send_goal(goal_pose)
		rospy.loginfo('sent goal')
		self.client.wait_for_result()

		#Write out time stamp and data at waypoints
		ts = time.time()
		st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
		self.outputWriter.writerow([self.sensor_reading_number,self.min_distance,st])
		self.sensor_reading_number = self.sensor_reading_number + 1

		#Interate waypoint number resets to 0 if on last waypoint, this is modified in the input varible
		#(userdata.patrol_waypoints_in) and out on the output (userdata.patrol_waypoint_num_out)
		if (len(userdata.patrol_waypoints_in) - 1) == userdata.patrol_waypoint_num_in:
			# print "reset to zero"
			userdata.patrol_waypoint_num_out = 0
		else:
			userdata.patrol_waypoint_num_out = userdata.patrol_waypoint_num_in + 1
		#Go to outcome 3 which is pack to the top of patrol state
		return 'outcome3'

	#Key_callback to read for r or h keypress
	def key_callback(self, keypress):
		if keypress.data == "r":
			self.client.cancel_goal()
			self.r_bool = True
			return 'outcome2'
		if keypress.data == "h":
			self.client.cancel_goal()
			self.h_bool = True
			return 'outcome1'

	#Callback for scan used for saving data at
	def scan_callback(self,scan):
		self.min_distance = min(scan.ranges)

	#Shutdown hook to close open csv file
	def shutdown_hook(self):
		self.outputFile.close()

#Shuffle State
class shuffle_state(smach.State):
	#Init Shuffle
	def __init__(self):
		#Define 1 outcome back to patrol, 2 inputs, 2 outputs
		smach.State.__init__(self,outcomes=['outcome1'],input_keys=['shuffle_waypoints_in','shuffle_waypoint_num_in'],output_keys=['shuffle_waypoints_out','shuffle_waypoint_num_out'])
	#Main funtion for shuffle state
	def execute(self, userdata):
		rospy.loginfo('Executing Shuffle')
		#Deep copy waypoint list
		shuffle_list = copy.deepcopy(userdata.shuffle_waypoints_in)
		index_shuffle = range(len(userdata.shuffle_waypoints_in))
		np.random.shuffle(index_shuffle)
		for i,w in enumerate(userdata.shuffle_waypoints_in):
			shuffle_list[i][0] = str(index_shuffle[i])
		sorted_shuffle_list = sorted(shuffle_list, key=lambda point_number: point_number[0])

		userdata.shuffle_waypoints_out = sorted_shuffle_list
		userdata.shuffle_waypoint_num_out = 0

		return 'outcome1'

#Reset State
class reset_state(smach.State):
	#Init Reset
	def __init__(self):
		#Define 1 outcome to patrol, 2 inputs and 1 output
		smach.State.__init__(self,outcomes=['outcome1'],input_keys=['reset_waypoints_in','reset_waypoint_num_in'],output_keys=['reset_waypoint_num_out'])
	def execute(self, userdata):
		#Reset waypoint number to 0
		rospy.loginfo('Reseting State Machine')
		userdata.reset_waypoint_num_out = 0
		return 'outcome1'

#main function
def main():
	#Init Node
	rospy.init_node('patrol_state_machine')

	#Open CSV File
	with open('/home/strider/catkin_ws/src/wagon_rut_costmap/scripts/output.csv', 'rb') as f:
		reader = csv.reader(f)
		for row in reader:
			waypoint_list.append(row)
		# print waypoint_list


	#Define Final State/Outcome of State machine (Not Used In This Code)
	patrol = smach.StateMachine(outcomes=['outcome5'])
	#Define "state" varible userdata.waypoints (no other definition need)
	patrol.userdata.waypoints = waypoint_list
	#Define "state" varible userdata.waypoint_number (no other definition need)
	patrol.userdata.waypoint_number = 0

	#Create patrol
	with patrol:
		#Add State Patrol, define transitions, remap varibles for patrol state
		smach.StateMachine.add('patrol',patrol_state(),transitions={'outcome1':'reset','outcome2':'shuffle','outcome3':'patrol','outcome4':'outcome5'},remapping={'patrol_waypoints_in':'waypoints','patrol_waypoint_num_in':'waypoint_number','patrol_waypoint_num_out':'waypoint_number'})
		#Add State Shuffle, define transitions, remap varibles for shuffle state
		smach.StateMachine.add('shuffle',shuffle_state(),transitions={'outcome1':'patrol'},remapping={'shuffle_waypoints_in':'waypoints','shuffle_waypoints_out':'waypoints','shuffle_waypoint_num_in':'waypoint_number','shuffle_waypoint_num_out':'waypoint_number'})
		#Add State Reset, define transitions, remap varibles for reset state
		smach.StateMachine.add('reset',reset_state(),transitions={'outcome1':'patrol'},remapping={'reset_waypoints_in':'waypoints','reset_waypoint_num_in':'waypoint_number','reset_waypoint_num_out':'waypoint_number'})
	#State machine creation
	outcome = patrol.execute()

if __name__ == '__main__':
	#Run Main Function
	main()
	#Shutdown Hook to close csv on shutdown
	rospy.on_shutdown(patrol_state.shutdown_hook())
	rospy.spin()
