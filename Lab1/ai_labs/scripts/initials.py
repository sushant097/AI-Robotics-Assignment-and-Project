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

print('Drawing My initial Name "SG" . . .')


# TODO: Modify the code below so that the robot moves in a square

# create two different Twist() variables.  One for moving forward.  One for turning 45 degrees.

# Forward
move_cmd = Twist()
move_cmd.linear.x= -1.5
move_cmd.angular.z = 0
# by default angular.z is zero

# turn at 45 deg
turn_cmd = Twist()
turn_cmd.linear.y=0
turn_cmd.linear.x = 0
turn_cmd.angular.z = -radians(45)

while not rospy.is_shutdown():

	# pub.publish(move_cmd)
	# rospy.sleep(2)
	
	move_cmd.linear.x = 1.5
	move_cmd.linear.y = 0
	pub.publish(move_cmd)
	rospy.sleep(2)

	move_cmd.linear.x = -1.5
	move_cmd.linear.y = 0
	pub.publish(move_cmd)
	rospy.sleep(2)

	move_cmd.linear.x = 0
	move_cmd.linear.y = -1.5
	pub.publish(move_cmd)
	rospy.sleep(2)

	move_cmd.linear.x = 1.5
	move_cmd.linear.y = 0
	pub.publish(move_cmd)
	rospy.sleep(2)

	move_cmd.linear.x = 0
	move_cmd.linear.y = -1.5
	pub.publish(move_cmd)
	rospy.sleep(2)

	move_cmd.linear.x = -1.5
	move_cmd.linear.y = 0
	pub.publish(move_cmd)
	rospy.sleep(2)

	# S is completed

	##########

	# Turn 45 degree
	turn_cmd.linear.y=0
	turn_cmd.linear.x = 0
	turn_cmd.angular.z = radians(45)
	pub.publish(turn_cmd)
	rospy.sleep(2)

	# For G
	move_cmd.linear.x = 4.3
	move_cmd.linear.y = 0
	move_cmd.angular.z = 0.0
	pub.publish(move_cmd)
	rospy.sleep(2)

	# Turn 45 degree
	turn_cmd.linear.y=0
	turn_cmd.linear.x = 0
	turn_cmd.angular.z = -radians(45)
	pub.publish(turn_cmd)
	rospy.sleep(2)

	# go forward
	move_cmd.linear.x = 1.5
	move_cmd.linear.y = 0
	move_cmd.angular.z = 0.0
	pub.publish(move_cmd)
	rospy.sleep(2)


	# come backward
	move_cmd.linear.x = -1.5
	move_cmd.linear.y = 0
	pub.publish(move_cmd)
	rospy.sleep(2)

	# down
	move_cmd.linear.x = 0
	move_cmd.linear.y = -3.0 # -1.5
	pub.publish(move_cmd)
	rospy.sleep(2)

	#forward
	move_cmd.linear.x = 1.5
	move_cmd.linear.y = 0
	pub.publish(move_cmd)
	rospy.sleep(2)
	
	# little up
	move_cmd.linear.x = 0
	move_cmd.linear.y = 0.25
	pub.publish(move_cmd)
	rospy.sleep(2)
	


	# extra forward and backward
	move_cmd.linear.x = 0.25
	move_cmd.linear.y = 0
	pub.publish(move_cmd)
	rospy.sleep(2)
	move_cmd.linear.x = -0.5
	move_cmd.linear.y = 0
	pub.publish(move_cmd)
	rospy.sleep(2)
	move_cmd.linear.x = 0.25
	move_cmd.linear.y = 0
	pub.publish(move_cmd)
	rospy.sleep(2)

	# down
	move_cmd.linear.x = 0
	move_cmd.linear.y = -1
	pub.publish(move_cmd)
	rospy.sleep(2)


    # G is completed
    
	break


rospy.spin()
    
    


    
