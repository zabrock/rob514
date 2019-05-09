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

# Global variables
global x_sensor_offset # Offset of sensor from robot center along x-axis
global robot_width # Width of the robot, with factor of safety for collisions
global deadzone # Deadzone in which no action is taken

robot_width = 0.33
deadzone = 0.005

# This is called every time we get a teleop command from ROS.
def teleop_callback(msg):
	
	# Simply save the incoming message for further manipulation during
	# a laser_callback
	global teleop_msg
	teleop_msg = msg


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
	# Run through the sensor measurements and find the smallest distance
	for i in range(0,len(ranges)):
		theta = i*delta_theta + angle_min
		if abs(theta) > theta_collide:
			continue
		linear_dist = ranges[i]*math.cos(theta)
		if not min_dist or abs(linear_dist) < abs(min_dist):
			min_dist = linear_dist

	# Calculate the difference between the minimum distance and
	# our desired safe wall distance
	dist_from_stop = min_dist - (stop_distance + x_sensor_offset)
	# Calculate a multiplier for the user-commanded velocity based
	# on the current distance from desired safe wall distance
	if abs(dist_from_stop) < deadzone:
		# If the robot is already close to safe wall distance,
		# allow no positive velocity input
		vel_multiplier = 0.0
	elif dist_from_stop < 0.0:
		# If the robot is closer than safe wall distance,
		# make the velocity negative and approx. scaled
		# to the current distance
		vel_multiplier = math.tanh(dist_from_stop)
	else:
		# Scale the user input according to current distance to wall
		vel_multiplier = math.tanh(3*dist_from_stop)

	if teleop_msg.linear.x > 0.0:
		# If the user is commanding positive velocity,
		# apply the velocity multiplier
		vel = teleop_msg.linear.x*vel_multiplier
	elif abs(teleop_msg.linear.x) < 0.005 and vel_multiplier < 0.0:
		# If the user is not commanding any velocity and current
		# distance to wall is less than safe, make velocity equal to
		# the multiplier
		vel = vel_multiplier
	else:
		# If the user commands negative velocity, let that pass through
		# without modification
		vel = teleop_msg.linear.x

	# Copy the teleop command and apply x-velocity scaling
	command = Twist()
	command = teleop_msg
	command.linear.x = vel

	# Publish the command using the global publisher
	pub.publish(command)


if __name__ == '__main__':
	rospy.init_node('guarded_teleop')

	# A subscriber for the laser scan data
	sub_scan = rospy.Subscriber('scan', LaserScan, laser_callback)

	# Subscriber for the turtlebot_teleop commands
	sub_teleop = rospy.Subscriber('teleop_command', Twist, teleop_callback)

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
			# If the sensor offset cannot be obtained, assume
			# no sensor offset so that the rest of the code can run
			x_sensor_offset = 0.0
			continue

	rospy.spin()
