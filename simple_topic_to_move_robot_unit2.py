#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('move_robot_node') #Initialize Node
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) #Create topic called '/cmd_vel' to publish through it
rate = rospy.Rate(2) 
move = Twist() #create instance from Twist
move.linear.x = 0.5 #Move the robot with a linear velocity in the x axis
move.angular.z = 0.5 #Move the with an angular velocity in the z axis

while not rospy.is_shutdown(): 
  pub.publish(move) #send the data through the publisher in order to move the robot
  rate.sleep()
