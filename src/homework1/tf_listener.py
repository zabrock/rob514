#!/usr/bin/env python
import rospy
import tf

if __name__ == '__main__':
	rospy.init_node('test_tf_listener')

	listener = tf.TransformListener()

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/base_laser_link','/base_link',rospy.Time(0))
			print '== Sensor to robot center == x: ' + str(trans[0]) + ' y: ' + str(trans[1])
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
