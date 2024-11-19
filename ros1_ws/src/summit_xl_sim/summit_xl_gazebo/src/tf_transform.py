#!/usr/bin/env python3  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Pose 


if __name__ == '__main__':
    rospy.init_node('base_tf_listener')

    listener = tf.TransformListener()
    pub = rospy.Publisher('right_summit/base_footprint_world', Pose, queue_size=10)
    pose = Pose()

    rate = rospy.Rate(1000.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/right_summit_odom', '/right_summit_base_footprint', rospy.Time(0))
            pose.position.x = trans[0]
            pose.position.y = trans[1]
            pose.position.z = trans[2]
            pose.orientation.x = rot[0]
            pose.orientation.y = rot[1]
            pose.orientation.z = rot[2]
            pose.orientation.w = rot[3]
            pub.publish(pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass