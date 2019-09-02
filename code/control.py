#!/usr/bin/env python

import rospy
import numpy as np
from race.msg import drive_param
from race.msg import pid_input

sim_rate = 10
kp = 14.0
ki = 0.0			 # ki -> extra credit
kd = 0.09
prev_error = 0.0
integral_error = 0.0  # used for ki -> extra credit
servo_offset = 18.5	 # zero correction offset in case servo is misaligned.
# arbitrarily initialized. 25 is not a special value. This code can input desired velocity from the user.
vel_input = 25.0

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)


def control(data):
    global prev_error
    global integral_error
    global vel_input
    global kp
    global ki
    global kd

    # Your code goes here
    # 1. Scale the error
    # 2. Apply the PID equation on error to compute steering
    # 3. Make sure the steering value is within bounds for talker.py
    msg = drive_param()
    msg.velocity = vel_input
    pid_error = data.pid_error
    error = pid_error * kp
    errordot = kd * (pid_error - prev_error)
    angle = error + errordot
    if angle > 100:
        angle = 100
    elif angle < -100:
        angle = -100
    prev_error = pid_error
    msg.angle = angle
    print(msg.angle)
    print("-------------")
    print(msg.velocity)
    pub.publish(msg)
    msg.angle = angle
    pub.publish(msg)
	# END


if __name__ == '__main__':
    global kp
    global kd
    global vel_input
    print("Listening to error for PID")
    kp = input("Enter Kp Value: ")
    ki = input("Enter Ki Value: ")
    kd = input("Enter Kd Value: ")
    vel_input = input("Enter Velocity: ")
    rospy.init_node('pid_controller', anonymous=True)
    rospy.Subscriber("error", pid_input, control)
    rospy.spin()
