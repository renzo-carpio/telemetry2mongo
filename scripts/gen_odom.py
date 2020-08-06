#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import random
import tf

def update_odom(odom):

    yaw= random.uniform(0, 1)/10
    d_x= random.uniform(0, 1)/10
    d_y= random.uniform(0, 1)/10

    odom.pose.pose.position.x = odom.pose.pose.position.x + d_x
    odom.pose.pose.position.y = odom.pose.pose.position.y + d_y

    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    odom.pose.pose.orientation.x = quaternion[0]
    odom.pose.pose.orientation.y = quaternion[1]
    odom.pose.pose.orientation.z = quaternion[2]
    odom.pose.pose.orientation.w = quaternion[3]

    return odom

def gen_odom():

    #ROS init
    rospy.init_node('gen_odom', anonymous=True)

    odom=Odometry()

    pub = rospy.Publisher('/ekf_localization_slam_node/slam_odom_magnetic', Odometry, queue_size=1)
    
    #main loop
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():

        odom=update_odom(odom)
        pub.publish(odom)
        rate.sleep()

if __name__ == '__main__':
    try:
        gen_odom()
    except rospy.ROSInterruptException:
        pass

