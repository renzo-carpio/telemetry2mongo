#!/usr/bin/env python

import rospy
from std_msgs.msg import String

global pub_string

def stringCallback(data):
    data_string=data
    pub_string.publish(data_string)

def payload_reader():

    global pub_string

    #ROS init
    rospy.init_node('pub_string', anonymous=True)

    pub_string = rospy.Publisher('/payload', String, queue_size=1)
    rospy.Subscriber('/payload_', String, stringCallback)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        payload_reader()
    except rospy.ROSInterruptException:
        pass

