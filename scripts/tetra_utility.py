#!/usr/bin/env python
import rospy
from sensor_msgs import joy
from tetraROS import compactOdom
from std_msgs.msgs import Int16
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

global current_time, last_time
mgmt_pub = rospy.publisher('cmd_mgmt', std_msgs.msg.Int16)
odom_broadcaster = tf.TransformBroadcaster()

def compactOdom_callback(odom_data)
    global current_time, last_time

    current_time = odom_data.stamp;
    #since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    #first, we'll publish the transform over tf
    odom_broadcaster.sendTransform((x, y, 0.), odom_quat, current_time, "base_link", "odom")
    
    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(x_speed, y_speed, 0), Vector3(0, 0, ang_speed))

    # publish the message
    odom_pub.publish(odom)

def joy_callback(joydata):
    if ((joydata.buttons[2]) == 1):
        rospy.loginfo("reset odometry")
        mgmt_pub.publish(1)

    if ((joydata.buttons[3]) == 1):
        rospy.loginfo("get battery level")
        mgmt_pub.publish(3)
         

def tetra_utility_loop():
    rospy.init_node('tetra_utility')
    rospy.suscriber('compact_odom', compactOdom, compactOdom_callback)
    rospy.suscriber('joy', joy, joy_callback)
    rospy.publisher('cmd_mgmt', std_msgs.msg.Int16)

    r = rospy.Rate(500) # 500hz
    while not rospy.is_shutdown():
        r.sleep()
    pass


if __name__ == '__main__':
    try:
        tetra_utility_loop()
    except rospy.ROSInterruptException:
        pass
