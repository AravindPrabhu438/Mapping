#!/usr/bin/env python
# above command need to be mentioned in every python scripts when using with ROS

#importing rospy to use ROS with python client 
import rospy
#importing the LaserScan data from the LaserSensor attached to the turtlebot. 
#We will subscribe to this laserscan data to build our obstacle avoidance algorithm 
from sensor_msgs.msg import LaserScan
# We import the Twist topic from the geometry_msgs to publish our messages to the turtlebot
from geometry_msgs.msg import Twist



def callback(msg):

#we access the ranges data from the from LaserScan and check if there is any obstable withing the distance of 0.5 meters 
	global move 
	if msg.ranges[0] > 0.5:
		#if there is no obstacle within 0.5 meters we command the turtlebot to move forward by publishing to linear.x
		move.linear.x = 0.5
		move.angular.z = 0.0

	else:
		#If there is an obstacle detected within 0.5 meters we command the turtlebot to turn by publishing to angular.z
		move.linear.x = -0.5
		move.angular.z = 0.5
	pub.publish(move)


#we create a node called listener
rospy.init_node('listener',anonymous = True)

#we create a publisher to publish on to /cmd_vel_mux/input/teleop which is equivalent to geometry_msgs/Twist for turtlebot
pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

# we created a subscriber to subscribe to the /scan data which provides the laser scan data of the turtlebot 

sub = rospy.Subscriber('/scan', LaserScan, callback)

move = Twist()
				
rospy.spin()
