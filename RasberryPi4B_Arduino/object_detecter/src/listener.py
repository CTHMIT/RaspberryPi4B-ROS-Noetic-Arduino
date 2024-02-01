#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("I heard Raspberry Pi talk to Arduino %s", data.data)
    
def checker():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("feedback", String, callback)

    rospy.spin()

if __name__ == '__main__':
    checker()
