#!/usr/bin/env python
import roslib
# roslib.load_manifest('intro_nav')
import rospy
from geometry_msgs.msg import PoseStamped
import csv
import time

class point_saver:
	def __init__(self):
		self.state_count = 0
		rospy.loginfo('Start Init')
		#Goal Subscriber
		self.move_base_sub = rospy.Subscriber('/move_base_simple/goal',PoseStamped,self.move_base_callback)
		self.point_array = []
		rospy.loginfo('End Init')
		#Open and write over output.csv
		self.outputFile = open('output.csv', 'w')
		self.outputWriter = csv.writer(self.outputFile)
		# self.outputWriter.writerow(['frame_id','x_point','y_point','point_z','x_quad','quad_y','z_quad','w_quad'])

		# with open('points.csv', 'w') as csvfile:
		# 	fieldnames = ['frame_id', 'x_point', 'y_point', 'z_point', 'x_quad', 'y_quad', 'z_quad', 'w_quad']
    	# 	self.writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
		# self.writer.writeheader()

	def move_base_callback(self, move_base_point):
		print "Point Found"
		frame_id = move_base_point.header.frame_id
		point_x = float("{0:.2f}".format(move_base_point.pose.position.x))
		point_y = float("{0:.2f}".format(move_base_point.pose.position.y))
		point_z = float("{0:.2f}".format(move_base_point.pose.position.z))

		quad_x = float("{0:.2f}".format(move_base_point.pose.orientation.x))
		quad_y = float("{0:.2f}".format(move_base_point.pose.orientation.y))
		quad_z = float("{0:.2f}".format(move_base_point.pose.orientation.z))
		quad_w = float("{0:.2f}".format(move_base_point.pose.orientation.w))

		# self.outputWriter.writerow({'frame_id':frame_id,'x_point':point_x,'y_point':point_y,'point_z':point_z,'x_quad':quad_x,'y_quad':quad_y,'z_quad':quad_z,'w_quad':quad_w})
		#Writing point to CSV
		self.outputWriter.writerow([self.state_count,frame_id,point_x,point_y,point_z,quad_x,quad_y,quad_z,quad_w])
		print frame_id
		#Incrementing Waypoint Number
		self.state_count = self.state_count + 1

		rospy.loginfo('callback')

	#Function To Be Run On Shutdown
	def shutdown_hook():
		self.outputFile.close()
		time.sleep(1)



if __name__ == '__main__':
	rospy.init_node('path_saver',log_level=rospy.DEBUG)
	point_saver = point_saver()
	#Call to function on node shutdown
	rospy.on_shutdown(point_saver.shutdown_hook)
	rospy.spin()
