#!/usr/bin/env python


import rospy
from sensor_msgs.msg import LaserScan

from random import gauss


def callback(msg):
	new_msg = LaserScan()

	# Make a new message, and copy in the information
	new_msg.header = msg.header
	new_msg.angle_min = msg.angle_min
	new_msg.angle_max = msg.angle_max
	new_msg.angle_increment = msg.angle_increment
	new_msg.time_increment = msg.time_increment
	new_msg.scan_time = msg.scan_time
	new_msg.range_min = msg.range_min
	new_msg.range_max = msg.range_max
	new_msg.intensities = msg.intensities[:]

	# Noise up the ranges
	new_msg.ranges = [gauss(r, 0.1) for r in msg.ranges]

	pub.publish(new_msg)


if __name__ == '__main__':
	rospy.init_node('noiser')

	pub = rospy.Publisher('/noisy_scan', LaserScan, queue_size=1)
	sub = rospy.Subscriber('/scan', LaserScan, callback)

	rospy.spin()
