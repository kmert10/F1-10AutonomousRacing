#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

							# Some useful variable declarations.
angle_range = 240		    # sensor angle range of the lidar
car_length = 1.5            # distance (in m) that we project the car forward for correcting the error. You may want to play with this.
desired_trajectory = 0.5    # distance from the wall (left or right - we cad define..but this is defined for right). You should try different values
vel = 5 		 		    # this vel variable is not really used here.
error = 0.0

pub = rospy.Publisher('error', pid_input, queue_size=10)

# -------------------------------------------------------------------------------
#   Input:  data: Lidar scan data
#           theta: The index of the angle at which the distance is required
#
#   OUTPUT: distance of scan at angle theta whose index is beam_index
# -------------------------------------------------------------------------------


def getRange(data, theta):
	# Find the index of the array that corresponds to angle theta.
	# Return the lidar scan value at that index
	distance = data.ranges[theta]
	car_theta = math.radians(theta) - math.pi / 2
	# Do some error checking for NaN and ubsurd values
	if car_theta > 3 * math.pi / 4:
		car_theta = 3 * math.pi / 4
	elif car_theta < -3 * math.pi / 4:
		car_theta = -3 * math.pi / 4
	float_index = (car_theta + 3 * math.pi / 4) / data.angle_increment
	index = int(float_index)
	return data.ranges[index]


def callback(data):
	theta = 50;
	a = getRange(data, theta)
	# Note that the 0 implies a horizontal ray..the actual angle for the LIDAR may be 30 degrees and not 0.
	b = getRange(data, 0)
	swing = math.radians(theta)
	# Your code goes here to compute alpha, AB, and CD..and finally the error.
	alpha = math.atan2(a * math.cos(swing) - b , a * math.sin(swing))
	AB = b * math.cos(alpha)
	AC = 1
	CD = AB + AC * math.sin(alpha)
	error = CD - desired_trajectory
	print "a {}\nb {}".format(a, b)
	print "AB {}".format(AB)
	print "error {}".format(error)
	# END

	msg = pid_input()
	msg.pid_error = error		# this is the error that you wantt o send to the PID for steering correction.
	msg.pid_vel = vel		# velocity error is only provided as an extra credit field.
	pub.publish(msg)


if __name__ == '__main__':
	print("Laser node started")
	rospy.init_node('dist_finder',anonymous = True)
	rospy.Subscriber("scan",LaserScan,callback)
	rospy.spin()
