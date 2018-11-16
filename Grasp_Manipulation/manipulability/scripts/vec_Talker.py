#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import MultiArrayLayout
import random

import numpy
def talker():
    pub = rospy.Publisher('Topic_Array', Float32MultiArray,queue_size=10)
    rospy.init_node('vecTalker', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        array = Float32MultiArray()
        array.layout.dim = 1
        for x in range(0, 90): 
        	array.data.append(random.randint(1,21)*5.1)
        my_array_for_publishing = Float32MultiArray(data=array)
        pub.publish(my_array_for_publishing)
        rospy.loginfo(str(my_array_for_publishing))
        pub.spin()
        #r.sleep()

if __name__ == '__main__':
    talker()


 ''' This is not working, you need to look into the examples below:
https://answers.ros.org/question/283413/custom-message/
https://www.programcreek.com/python/example/86349/std_msgs.msg.Float32
http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
http://wiki.ros.org/rospy_tutorials/Tutorials/numpy
https://stackoverflow.com/questions/31369934/ros-publishing-array-from-python-node
https://gist.github.com/alexsleat/1372845#file-subscribe-cpp
https://gist.github.com/alexsleat/1372845
https://stackoverflow.com/questions/33457705/how-to-publish-a-message-of-type-vectorpoint2d-to-topic-in-ros
https://answers.ros.org/question/43157/trying-to-use-get-joint-state-with-my-urdf/
'''