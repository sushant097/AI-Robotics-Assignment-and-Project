#!/usr/bin/python3
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from tf.transformations import euler_from_quaternion
import math

#####################
# BEGIN Global Variable Definitions

robot = [0,0,0]
laser_scan = None
goal = None

# END Global Variable Definitions
#####################+

#####################
# BEGIN ROS Topic Callback Functions [DON'T MESS WITH THIS STUFF]
#####################

def robot_callback(data):
    #This function updates the robots position and yaw, based on the ground truth (we don't have localization yet)
    global robot
    [r,p,yaw] = euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
    robot = [data.pose.pose.position.x,data.pose.pose.position.y,yaw]

def laser_callback(data):
    #This function sets the global laser_scan variable to hold the most recent laser scan data
    global laser_scan
    laser_scan = data

def goalCallback(data):
    #This function will update the goal of the robot
    global goal
    goal = [data.x, data.y]

#####################
##### END CALLBACK FUNCTIONS   
#####################

##################### 
# BEGIN HELPER FUNCTIONS [USE THESE IN YOUR CODE, BUT YOU SHOULDN'T NEED TO MODIFY]
#####################

def add_forces(a, b):
    #This function adds to force vectors together and returns the result
    assert len(a) == len(b), "Force vectors differ in length"
    c = [a[i] + b[i] for i in range(len(a))]
    return c

def wrap_angle(angle):
    #This function will take any angle and wrap it into the range [-pi, pi]
    while angle >= math.pi:
        angle = angle - 2*math.pi
        
    while angle <= -math.pi:
        angle = angle + 2*math.pi
    return angle

#####################
##### END HELPER FUNCTIONS
#####################

#####################
# BEGIN MODIFIABLE LAB CODE [ALTHOUGH MOST MODIFICATIONS SHOULD BE WHERE SPECIFIED]
#####################

#This function takes in a force [x,y] (in robot coordinates) and returns the drive command (Twist) that should be sent to the robot motors
def drive_from_force(force):
    
    #####################################################
    #PARAMETERS : MODIFY TO GET ROBOT TO MOVE EFFECTIVELY
    
    #This is multiplied by the angle of the drive force to get the turn command 
    turn_multiplier = 1.0
    
    #If the absolute value of the angle of the force direction is greater than this, we only spin
    spin_threshold = math.pi
    
    #This is multiplied by the magnitude of the force vector to get the drive forward command
    drive_multiplier = 1.0 
    
    #END OF PARAMETERS
    #####################################################

    #The twist command to fill out and return
    twist = Twist()

    #Determine the angle and magnitude of the force
    force_angle = math.atan2(force[1],force[0])
    force_mag = math.hypot(force[0],force[1])

    #Get turn speed
    twist.angular.z = turn_multiplier * force_angle

    #Do we just spin
    if abs(force_angle) < spin_threshold:
        twist.linear.x = drive_multiplier * force_mag

    return twist

# This function determines and returns the attractive force (force_to_goal) to the goal.  
# This force should be in robot coordinates
def goal_force( ):
        
    #This is the robot's actual global location, set in robot_callback
    global robot #format [x_position, y_position, yaw]

    #Goal location is in the global 'goal' variable
    global goal

    #####################################################
    #PARAMETERS : MODIFY TO GET ROBOT TO MOVE EFFECTIVELY
    
    #Parameter : MODIFY
    #This should be used to scale the magnitude of the attractive goal force
    strength = 1.0 
    
    #END OF PARAMETERS
    #####################################################
    
    force_to_goal = [0,0]    
    
    #########################
    # PART A : BEGIN
    #########################

    # Check if the goal is set
    if goal is not None:
        # Compute relative position of the goal from the robot's position
        goal_rel_x = goal[0] - robot[0]
        goal_rel_y = goal[1] - robot[1]

        # Robot's orientation (yaw)
        robot_yaw = robot[2]

        # Convert goal position to robot's coordinate system 
        # Rotate the goal position by negative of the robot's yaw
        goal_in_robot_x = math.cos(-robot_yaw) * goal_rel_x - math.sin(-robot_yaw) * goal_rel_y
        goal_in_robot_y = math.sin(-robot_yaw) * goal_rel_x + math.cos(-robot_yaw) * goal_rel_y

        # Scale the force vector
        force_to_goal = [strength * goal_in_robot_x, strength * goal_in_robot_y]

    # PART A CODE HERE: 
    #    1. Compute goal force vector and put it in the 'force_to_goal' variable

    #########################
    # PART A : END
    #########################
    
    return force_to_goal


#This function looks at the current laser reading, then computes and returns the obstacle avoidance force vector (in local robot coordinates)
def obstacle_force():  

    #The most recent laser_scan.  It has the following fields 
    #   laser_scan.angle_min : angle of the first distance reading
    #   laser_scan.angle_increment : the angular difference between consecutive distance readings
    #   laser_scan.ranges : an array all of the distance readings
    global laser_scan
    
    #Only run if we have a laser scan to work with
    if laser_scan is None:
        return [0,0]

    #The obstacle repulsion force variable, will be returned
    #This will accumulate all of the obstacle forces acting upon us
    force_from_obstacles = [0,0]

    cur_angle = laser_scan.angle_min 
    #cur_angle will always have the relative angle between the robot's yaw and the current laser reading

    for i in range(len(laser_scan.ranges)):

        # Get the magnitude of the repulsive force for this distance reading
        # CHANGE WHICH FUNCTION IS CALLED FOR  PART C
        strength = get_pf_magnitude_constant(laser_scan.ranges[i])
    
        #########################
        # PART B : BEGIN
        #########################

        # PART B CODE HERE: 
        #    1. Compute force vector with magnitude 'strength' away from obstacle
        force_x = -strength * math.cos(cur_angle)
        force_y = -strength * math.sin(cur_angle)


        #    2. Add this force vector to the 'force_from_obstacles' vector
        force_from_obstacles[0] += force_x
        force_from_obstacles[1] += force_y

        #########################
        # PART B : END
        #########################

        cur_angle = cur_angle + laser_scan.angle_increment

    return force_from_obstacles

# This function returns the magnitude of repulsive force for the input distance
# using a linear drop-off function
def get_pf_magnitude_linear(distance):

    #####################################################
    #PARAMETERS: MODIFY TO GET THINGS WORKING EFFECTIVELY
        
    #How close to the obstacle do we have to be to begin feeling repulsive force
    distance_threshold = 1.0 

    #The maximum strength of the repulsive force
    max_strength = 1.0

    #END OF PARAMETERS
    #####################################################
    
    #########################
    #  PART C : BEGIN
    #########################
    # If the distance is greater than the threshold, no force is applied
    if distance >= distance_threshold:
        return 0

    # PART C CODE HERE: 
    #   1. Compute the magnitude of the force for the given distance and return it

    # Linear drop-off of force magnitude
    # As the distance increases, the force decreases linearly
    return max_strength * (1 - distance / distance_threshold)
    #########################
    #  PART C : END
    #########################



# This function returns the magnitude of repulsive force for the input distance
# using a constant value if the obstacles is closer than a threshold
def get_pf_magnitude_constant(distance):

    #####################################################
    #PARAMETERS: MODIFY TO GET THINGS WORKING EFFECTIVELY
        
    #How close to the obstacle do we have to be to begin feeling repulsive force
    distance_threshold = 1.0 

    #Strength of the repulsive force
    strength = 1.0

    #END OF PARAMETERS
    #####################################################

    if distance < distance_threshold:
        return strength

    return 0

# This is the main loop of the lab code.  It runs continuously, navigating our robot
# (hopefully) towards the goal, without hitting any obstacles
def potential():
    rospy.init_node('potential_fields', anonymous=True) #Initialize the ros node
    pub = rospy.Publisher('cmd_vel', Twist) #Create our publisher to send drive commands to the robot
    rospy.Subscriber("base_scan", LaserScan, laser_callback) #Subscribe to the laser scan topic
    rospy.Subscriber("base_pose_ground_truth", Odometry, robot_callback) #Subscribe to the robot pose topic
    rospy.Subscriber("next_waypoint", Point, goalCallback)#Subscribe to the goal location topic

    rate = rospy.Rate(10) #10 Hz    
    
    while not rospy.is_shutdown():
        
        #Don't do anything until the goal location has been received
        if goal is None:
            rate.sleep()
            continue

        #1. Compute attractive force to goal
        g_force = goal_force()
        
        #2. Compute obstacle avoidance force
        o_force = obstacle_force()

        #3. Get total force by adding together
        total_force = add_forces(g_force, o_force)
        
        #4. Get final drive command from total force
        twist = drive_from_force(total_force) 

        #5. Publish drive command, then sleep 
        pub.publish(twist)
        
        rate.sleep() #sleep until the next time to publish
        

    #Send empty twist command to make sure robot stops
    twist = Twist()
    pub.publish(twist)

if __name__ == '__main__':
    try:
        potential()
    except rospy.ROSInterruptException:
        pass
