#!/usr/bin/env python
import rospy
import numpy
from geometry_msgs.msg import Twist 
from math import radians

# Creates a node with name 'square' and
# unique node (anonymous=True)
rospy.init_node('square', anonymous=True)
# Publisher which will publish to the topic '/turtle1/cmd_vel'
pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(10) # 10hz

print('Starting square . . .')


# TODO: Modify the code below so that the robot moves in a square

# create two different Twist() variables.  One for moving forward.  One for turning 45 degrees.

# Forward
move_cmd = Twist()
move_cmd.linear.x= 2.2
move_cmd.angular.z = 0
# by default angular.z is zero

# turn at 45 deg
turn_cmd = Twist()
turn_cmd.linear.y=0
turn_cmd.linear.x = 0
turn_cmd.angular.z = radians(90)

while not rospy.is_shutdown():
	for i in range(0, 5):
		rospy.loginfo("Going Straight")
		pub.publish(move_cmd)
		rospy.sleep(5)


		# turn 90 degrees
		rospy.loginfo("Turning")
		pub.publish(turn_cmd)
		rospy.sleep(2)
	
	rospy.loginfo("Done!")
	move_cmd.linear.x = 0.0
	move_cmd.angular.z = 0.0
	pub.publish(move_cmd)
    
	break


rospy.spin()
    
    


    
