#!/usr/bin/env python

#Author Joseph St. Germain 
#Co-Authur Arthur lockmans drive smooth function


import rospy, tf, numpy, math
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion


wheel_rad = 3.5 / 100.0 #cm
wheel_base = 23.0 / 100.0 #cm

def publishTwist(lin_Vel, ang_Vel):
    """Send a movement (twist) message."""
    global pub
    msg = Twist()
    msg.linear.x = lin_Vel
    msg.angular.z = ang_Vel
    pub.publish(msg)


def navToPose(goal):
    """Drive to a goal subscribed to from /move_base_simple/goal"""
    #compute angle required to make straight-line move to desired pose
    global xPosition
    global yPosition
    global theta
    #capture desired x and y positions
    desiredY = goal.pose.position.y
    desiredX = goal.pose.position.x
    #capture desired angle
    quat = goal.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    desiredT = yaw * (180.0/math.pi)
    #compute distance to target
##############
    #compute initial turn amount
##############


    print "spin!" #turn to calculated angle
    rotate(initialTurn)
    print "move!" #move in straight line specified distance to new pose
    driveSmooth(0.25, distance)
    rospy.sleep(2)
    print "spin!" #spin to final angle 
    print "done"



#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
    """This function accepts two wheel velocities and a time interval."""
    global pub

    r = wheel_rad
    b = wheel_base
    #compute wheel speeds
    u = #############     #Determines the linear velocity of base based on the wheel
    w = #############     #Determines the angular velocity of base on the wheels.
    start = rospy.Time().now().secs
    #create movement and stop messages
    move_msg = Twist() #creates a move_msg object inheriting type from the Twist() class
    move_msg.linear.x = u #sets linear velocity
    move_msg.angular.z = w #sets amgular velocity (Populates messages with data.)

    stop_msg = Twist()
    stop_msg.linear.x = 0
    stop_msg.angular.z = 0
    #publish move message for desired time
    while(rospy.Time().now().secs - start < time and not rospy.is_shutdown()): # waits for said time and checks for ctrl C
        pub.publish(move_msg) #publishes the move_msg
    pub.publish(stop_msg)

#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    """This function accepts a speed and a distance for the robot to move in a straight line"""
    global pose

    initialX = pose.pose.position.x
    initialY = pose.pose.position.y
    atTarget = False
    #Loop until the distance between the attached frame and the origin is equal to the
    #distance specified 
    while (not atTarget and not rospy.is_shutdown()):
        currentX = pose.pose.position.x
        currentY = pose.pose.position.y
        #currentDistance = math.sqrt(xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx)#Distance formula
        if (currentDistance >= distance):
            atTarget = True
            publishTwist(0, 0)
        else:
            publishTwist(speed, 0)
            rospy.sleep(0.15)


def driveSmooth(speed, distance):
    """This function accepts a speed and a distance for the robot to move in a smoothed straight line."""



def rotate(angle):
    global odom_list
    global pose
    if (angle > 180 or angle<-180):
        print "angle is to large or small"
    vel = Twist();   
    done = True

    # set rotation direction
    error = angle-math.degrees(pose.orientation.z)
#####################
    #determine which way to turn based on the angle
#######################

    while ((abs(error) >= 2) and not rospy.is_shutdown()):
        #Use this while loop to start the robots motion and determine if you are at the right angle.    
        #print "theta: %d" % math.degrees(pose.orientation.z)
        #################    
    vel.angular.z = 0.0
    pub.publish(vel)


def executeTrajectory():
    """This function sequentially calls methods to perform a trajectory."""
    pass



def driveArc(radius, speed, angle):
    """This function works the same as rotate how ever it does not publish linear velocities."""
    #assuming radius is turning radius, speed is drive speed, angle is desired final angle
    #calculate wheel speeds and time to move from current pose to final pose
    #spinWheels with time and speeds to move to correct pose
    w = speed / radius
    v1 = w * (radius + .5*.352)
    v2 = w * (radius - .5*.352)

    ############################# The rest of this function will be at least as long as rotate
    pass  # Delete this 'pass' once implemented


def readBumper(msg):
    """Bumper event callback"""
    if (msg.state == 1):
        # What should happen when the bumper is pressed?
        #Stop forward motion if bumper is pressed
        print "Bumper pressed!"
        executeTrajectory()


#keeps track of current location and orientation
def tCallback(event):
	
    global pose
    global xPosition
    global yPosition
    global theta

    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
    (position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0))
    pose.position.x=position[0]
    pose.position.y=position[1]
    # the previous 2 lines and next 2 lines are repedative. Joes bad
    xPosition=position[0]
    yPosition=position[1]

    odomW = orientation
    q = [odomW[0], odomW[1], odomW[2], odomW[3]]
    roll, pitch, yaw = euler_from_quaternion(q)
    #convert yaw to degrees
    pose.orientation.z = yaw
    theta = math.degrees(yaw)



# This is the program's main function
if __name__ == '__main__':
    rospy.init_node('Joes_Lab_2_example')
    global pub
    global pose
    global odom_list
    #global odom_tf
    pose = Pose()
    pub = rospy.Publisher('###########', Twist, None, queue_size=10) # Publisher for commanding robot motion
    bumper_sub = rospy.Subscriber('###########', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
    goal_sub = rospy.Subscriber('##############', PoseStamped, navToPose, queue_size=1)

    rospy.Timer(rospy.Duration(.01), tCallback) # timer callback for robot location
    
    odom_list = tf.TransformListener() #listner for robot location

    rospy.sleep(2)

    print "Starting Lab 2"

    while not rospy.is_shutdown():
        rospy.spin()
    
    print "Lab 2 complete!"
