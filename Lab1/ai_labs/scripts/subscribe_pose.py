#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose


# Creates a node with name 'square' and
# unique node (anonymous=True)
rospy.init_node('subscribe_pose', anonymous=True)



def pose_callback(msg):
	rospy.loginfo("Robot Pose: ")
	rospy.loginfo("Pose(x, y, theta): (%f, %f, %f)", msg.x, msg.y, msg.theta)
	

# Publisher which will publish to the topic '/turtle1/pose'
pose_subscriber = rospy.Subscriber('turtle1/pose', Pose, pose_callback)

# while not rospy.is_shutdown():
# 	# pub.subscribe()
# 	# rospy.sleep(5)
rospy.spin()



    


    
