#!/usr/bin/env python
import rospy
from sensor_msgs import joy
from tetraROS import compactOdom
from std_msgs.msgs import Int16


mgmt_pub = rospy.publisher('cmd_mgmt', std_msgs.msg.Int16)

def compactOdom_callback(odom_data):
    pass


def joy_callback(joydata):
    if ((joydata.buttons[2]) == 1):
        rospy.loginfo("reset odometry")
        mgmt_pub.publish(1)

    if ((joydata.buttons[3]) == 1):
        rospy.loginfo("get battery level")
        mgmt_pub.publish(3)
        pass


def tetra_utility_loop():
    rospy.init_node('tetra_utility')
    rospy.suscriber('compact_odom', compactOdom, compactOdom_callback)
    rospy.suscriber('joy', joy, joy_callback)
    rospy.publisher('cmd_mgmt', std_msgs.msg.Int16)

    r = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        
        r.sleep()

    pass


if __name__ == '__main__':
    try:
        tetra_utility_loop()
    except rospy.ROSInterruptException:
        pass
