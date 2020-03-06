#!/usr/bin/env python
import rospy
from sensor_msgs import joy
from tetraROS import compactOdom


def compactOdom_callback(odom_data):
    pass


def joy_callback(joydata):
    if ((joydata.buttons[rospy.get_param('~reset_button', 1)]) == 1):
        rospy.loginfo("reset odometry")
        pass


def tetra_utility_loop():
    rospy.init_node('tetra_utility')
    rospy.suscriber('compact_odom', compactOdom, compactOdom_callback)
    rospy.suscriber('joy', joy, joy_callback)

    pass


if __name__ == '__main__':
    try:
        tetra_utility_loop()
    except rospy.ROSInterruptException:
        pass
