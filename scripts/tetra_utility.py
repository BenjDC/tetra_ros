#!/usr/bin/env python
import rospy
from tetra_ros.msg import compactOdom
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy


mgmt_pub = rospy.Publisher('cmd_mgmt', Int16, queue_size=10)

def compactOdom_callback(odom_data):
    pass


def joy_callback(joydata):
    if ((joydata.buttons[2]) == 1):
        rospy.logdebug("reset odometry")
        mgmt_pub.publish(1)

    if ((joydata.buttons[3]) == 1):
        rospy.logdebug("get battery level")
        mgmt_pub.publish(3)
        pass


def tetra_utility_loop():
    rospy.init_node('tetra_utility')
    rospy.Subscriber('compact_odom', compactOdom, compactOdom_callback)
    rospy.Subscriber('joy', Joy, joy_callback)

    r = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        
        r.sleep()

    pass


if __name__ == '__main__':
    try:
        tetra_utility_loop()
    except rospy.ROSInterruptException:
        pass
