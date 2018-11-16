#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import MultiArrayLayout



def callback(data):
    print rospy.get_name(), "I heard %s"%str(data.data)
    #print msg

#ef callback(data):
 #  print rospy.get_name(), "I heard %s"%str(data.data)

def listener():
    rospy.init_node('vecListener')
    
    while not rospy.is_shutdown():
        #r = rospy.Rate(0.5)
        #print "Before Subscriber"
        rospy.Subscriber("Topic_Array", Float32MultiArray, callback)
        #print "After Subscriber:"
        rospy.spin()


if __name__ == '__main__':
    listener()