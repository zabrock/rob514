#!/usr/bin/env python


import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import os.path

import message_filters
from sensor_msgs.msg import LaserScan

from math import sin,cos


# This callback synchronizes odometry and lidar messages.  It also calculates the heading of the robot as an Euler angle, so 
# that it's easier to use with trigonometric functions.
def callback(odom_msg, lidar_msg):
	(_, _, heading) = euler_from_quaternion([odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w])
	print 'Pose: ', odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, heading
	print 'Lidar[0]', lidar_msg.ranges[0]

	# Go through the ranges in LaserScan and determine the x,y coordinates
	# from the camera frame
	min_lidar_angle = lidar_msg.angle_min
	lidar_angle_delta = lidar_msg.angle_increment
	tol = 0.01
	x = []
	y = []
	for i in range(0,len(lidar_msg.ranges)):
		# Only look at points with high intensity
		if lidar_msg.intensities[i] > 0.25:
			# Calculate angle of current measurement
			lidar_angle = min_lidar_angle + i*lidar_angle_delta
			# Calculate and save x and y coordinates
			x_point = odom_msg.pose.pose.position.x + lidar_msg.ranges[i]*cos(lidar_angle+heading)
			y_point = odom_msg.pose.pose.position.y + lidar_msg.ranges[i]*sin(lidar_angle+heading)
			x.append(x_point)
			y.append(y_point)

	# After calculating all the coordinates in this measurement,
	# write the coordinates to a text file
	with open('world_points.txt','a') as file:
		for i in range(0,len(x)):
			file.write(str(x[i]) + ',' + str(y[i]) + '\n')


if __name__ == '__main__':
	rospy.init_node('mapper')

	if os.path.isfile('world_points.txt'):
		os.remove('world_points.txt')

	odom_sub = message_filters.Subscriber('/odom', Odometry)
	lidar_sub = message_filters.Subscriber('/scan', LaserScan)
	sub = message_filters.TimeSynchronizer([odom_sub, lidar_sub], 10)
	sub.registerCallback(callback)

	rospy.spin()


