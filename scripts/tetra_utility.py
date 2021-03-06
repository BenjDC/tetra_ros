#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from tetra_ros.msg import compactOdom
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import math


mgmt_pub = rospy.Publisher('cmd_mgmt', Int16, queue_size=10)
odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()
laser_broadcaster = tf.TransformBroadcaster()




def compactOdom_callback(odom_data):

    # since all odometry is 6DOF,
    # we'll need a quaternion created from yaw, for odometry and laser
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, math.radians(odom_data.ang_pos))
    laser_quat = tf.transformations.quaternion_from_euler(0, 0, 0)

    # first, we'll publish the odom transform over tf
    odom_broadcaster.sendTransform((odom_data.x_pos, odom_data.y_pos, 0.), odom_quat, odom_data.stamp, "base_link", "odom")
    # the laser tf is 3 cm +x and 10cm +z
    laser_broadcaster.sendTransform((0.03, 0., 0.10), laser_quat, odom_data.stamp, "laser", "base_link")

    # odom and map are the same ?
    # laser_broadcaster.sendTransform((0.00, 0., 0.0), laser_quat, odom_data.stamp, "odom", "map")

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = odom_data.stamp
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(odom_data.x_pos, odom_data.y_pos, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(odom_data.x_speed, odom_data.y_speed, 0), Vector3(0, 0, math.radians(odom_data.ang_speed)))

    # publish the message
    odom_pub.publish(odom)


def joy_callback(joydata):

    if ((joydata.buttons[2]) == 1):
        rospy.loginfo("reset odometry")
        mgmt_pub.publish(1)

    if ((joydata.buttons[3]) == 1):
        rospy.loginfo("get battery level")
        mgmt_pub.publish(3)

    if ((joydata.buttons[9]) == 1):
        rospy.loginfo("KILL NOW")
        rospy.signal_shutdown("Kill from controller ")

    
def tetra_utility_loop():
    rospy.init_node('tetra_utility', disable_signals=True)
    rospy.Subscriber('compact_odom', compactOdom, compactOdom_callback)
    rospy.Subscriber('joy', Joy, joy_callback)

    r = rospy.Rate(500)  # 500hz

    rospy.loginfo("Welcome to tetra_ros ! here are available commands from the PS3 DualShock")
    rospy.loginfo("     ▲  : reset odometry")
    rospy.loginfo("     X  : Enable left joystick for direction")
    rospy.loginfo("     □  : Get battery level")
    rospy.loginfo("     START : Kill all nodes")


    while not rospy.is_shutdown():
        r.sleep()
    pass


if __name__ == '__main__':
    try:
        tetra_utility_loop()
    except rospy.ROSInterruptException:
        pass
