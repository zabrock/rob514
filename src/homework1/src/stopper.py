#!/usr/bin/env python

# Every python controller needs these lines
import rospy

# The velocity command message
from geometry_msgs.msg import Twist

# The laser scan message
from sensor_msgs.msg import LaserScan

# The TF message
import tf

# Used for angular calculations
import math

# Global variable for the sensor offset from the center of the robot
global x_sensor_offset # Offset of sensor from robot center along x-axis
global robot_width # Width of the robot, with factor of safety for collisions
global deadzone # Deadzone in which no action is taken

robot_width = 0.33
deadzone = 0.005

# This is called every time we get a LaserScan message from ROS.
def laser_callback(msg):

	# Get the angle data from the sensor
	delta_theta = msg.angle_increment
	angle_min = msg.angle_min
	ranges = msg.ranges

	# Retrieve the desired stop distance
	stop_distance = rospy.get_param('stop_distance',0.5)
	

	# Determine the angle range we care about
	theta_collide = math.atan2(robot_width/2,stop_distance)
	min_dist = []
	# Run through each of the angles we care about and find
	# the smallest distance
	for i in range(0,len(ranges)):
		theta = i*delta_theta + angle_min
		if abs(theta) > theta_collide:
			continue
		linear_dist = ranges[i]*math.cos(theta)
		if not min_dist or abs(linear_dist) < abs(min_dist):
			min_dist = linear_dist
	error = min_dist - (stop_distance + x_sensor_offset)
	if abs(error) < deadzone:
		vel = 0.0
	else:
		vel = 0.2*math.tanh(3*error)

	# Drive forward at the desired velocity
	command = Twist()
	command.linear.x = vel
	command.linear.y = 0.0
	command.linear.z = 0.0
	command.angular.x = 0.0
	command.angular.y = 0.0
	command.angular.z = 0.0

	# Publish the command using the global publisher
	pub.publish(command)



if __name__ == '__main__':
	rospy.init_node('stopper')

	# A subscriber for the laser scan data
	sub = rospy.Subscriber('scan', LaserScan, laser_callback)

	# A publisher for the move data
	pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

	# Get the distance from the laser sensor to the center of the robot
	listener = tf.TransformListener()
	tf_received = False
	while not tf_received:
		try:
			(trans,rot) = listener.lookupTransform('/base_laser_link','/base_link',rospy.Time(0))
			tf_received = True
			x_sensor_offset = trans[0]
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			# If the transform can't be found, assume no sensor offset
			x_sensor_offset = 0.0
			continue

	rospy.spin()
