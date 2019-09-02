#!/usr/bin/env python

import rospy
from race.msg import drive_values
from race.msg import drive_param
from std_msgs.msg import Bool

def ardu_map(x, in_lower, in_upper, out_lower, out_upper):
    return (x - in_lower) * (out_upper - out_lower) // (in_upper - in_lower) + o
ut_lower

class Talker():
    def __init__(self):
        rospy.init_node('Talker', anonymous=False)
        self.cmd = drive_values()
        self.pub = rospy.Publisher('drive_pwm', drive_values, queue_size=10)
        self.em_pub = rospy.Publisher('eStop', Bool, queue_size=10)
        self.sub = rospy.Subscriber('drive_parameters', drive_param, self.update
_cmd)
        self.rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.rate.sleep()
    def update_cmd(self, data):
        rospy.loginfo(data)
        self.cmd.pwm_drive = ardu_map(data.velocity,-100,100,6554,13108)
        self.cmd.pwm_angle = ardu_map(data.angle, -100, 100, 6554, 13108)
        rospy.loginfo(self.cmd)
        self.pub.publish(self.cmd)
        self.em_pub.publish(False)


if __name__ == '__main__':
    Talker()


