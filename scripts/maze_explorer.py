#! /usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_msgs.msg import String
import time
PI = 3.14

def stop(vel_msg, velocity_publisher):  
	vel_msg.linear.x = 0 #set linear x to zero  
	vel_msg.angular.z = 0 #set angular z to zero  
	velocity_publisher.publish(vel_msg) #Publish the velocity to stop the robot
	time.sleep(1) #stop for 1 second
	
def movingForward(vel_msg, velocity_publisher, t0, current_distance, distance, speed, forwardSpeed, front):
	vel_msg.linear.x = forwardSpeed  
	vel_msg.angular.z = 0 #initialize angular z to zero   
	print('Is moving')
	while(current_distance < distance):
		velocity_publisher.publish(vel_msg)#Publish the velocity		 
		t1 = rospy.Time.now().to_sec()   #Take actual time to velocity calculation
		current_distance = speed*(t1-t0)#calculates distance
	if (front < 0.75):
		stop(vel_msg, velocity_publisher) #Stop the robot when the front distance from obstacle is smaller than
	time.sleep(1) # stop for 1 second
	
def movingBackward(vel_msg, velocity_publisher, t0, current_distance, distance, speed, backwardSpeed, front):
	vel_msg.linear.x = backwardSpeed
	vel_msg.angular.z = 0 #initialize angular z to zero  print('Is backing')  
	while(current_distance < (distance/2)):
		velocity_publisher.publish(vel_msg)#Publish the velocity
		#Take actual time to vel calculation  
		t1 = rospy.Time.now().to_sec()
		current_distance = speed*(t1-t0)#calculates distance  
		if (front < 0.75):
			stop(vel_msg, velocity_publisher)  
		time.sleep(1) # stop for 1 second
		
def turnCW(vel_msg, velocity_publisher, t0, current_angle, turningSpeed, angle):   #converting from angle to radian
	angular_speed = round(turningSpeed*2*PI/360, 1)
	relative_angle = round(angle*2*PI/360, 1)   
	vel_msg.linear.x = 0 #initialize linear x to zero
	vel_msg.angular.z = -abs(angular_speed)
	print('Turning')   
	while(current_angle < relative_angle):
		velocity_publisher.publish(vel_msg)#Publish the velocity  
		#Take actual time to vel calculation
		t1 = rospy.Time.now().to_sec()
		current_angle = angular_speed*(t1-t0)#calculates distance
		time.sleep(1)#stop the robot for 1 second
		
def turnCCW(vel_msg, velocity_publisher, t0, current_angle, turningSpeed, angle):   #converting from angle to radian
	angular_speed = round(turningSpeed*2*PI/360, 1)
	relative_angle = round(angle*2*PI/360, 1)   
	vel_msg.linear.x = 0 #initialize linear x to zero
	vel_msg.angular.z = abs(angular_speed)
	print('Turning')
	while(current_angle < relative_angle):
		velocity_publisher.publish(vel_msg)#Publish the velocity  
		t1 = rospy.Time.now().to_sec()
		current_angle = angular_speed*(t1-t0)#calculates distance\
		time.sleep(1)#stop the robot for 1 second
		
def escapeMaze():
	rospy.init_node('escapeMaze', anonymous=True)
	velocity_publisher = rospy.Publisher('cmd_vel', Twist,queue_size=10)
	vel_msg = Twist()
	print("Let's move the robot")  
	#define the local speed 
	speed = 0.2   #define the distance
	distance = [0.02, 0.28, 0.4]   #set all the linear and angular motion of each dimension to zero				
	vel_msg.linear.x = 0
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = 0

	while not rospy.is_shutdown():
		no_right_wall = None #define the variable to determine the existance of the front wall
		no_front_wall = None #set current time for distance calculate  
		no_left_wall = None
		t0 = rospy.Time.now().to_sec() #define the variable for current distance
		current_distance = 0 #read the input from lazer scan
		scan_msg = rospy.wait_for_message("scan", LaserScan)
		front = scan_msg.ranges[1]
		left = scan_msg.ranges[90]
		top_left = scan_msg.ranges[45]
		right = scan_msg.ranges[275]
		top_right = scan_msg.ranges[315] #direction is smaller than 2.0, else right wall is detected

		if front > 1.0:
			movingForward(vel_msg, velocity_publisher, t0, current_distance, distance[1], speed, 0.20, front)
		else:
			if (right< 2.0) or (top_right < 2.0):
				no_right_wall = False #False becuase right wall is detected   
				print('Right wall is detected')#display message
			else:
				no_right_wall = True #True becuase right wall is not detected		   
				print('Right wall is not detected') #display message

			if (left< 2.0) or (top_left < 2.0):
				no_left_wall = False #False becuase left wall is detected   
				print('Left wall is detected')#display message
			else:
				no_left_wall = True #True becuase left wall is not detected		   
				print('Left wall is not detected') #display message

			if no_right_wall == True and no_left_wall == False:
				movingBackward(vel_msg, velocity_publisher, t0, current_distance, distance[1], speed, -0.1, front)
				turnCW(vel_msg, velocity_publisher, t0, 0, 45, 30) 
			elif no_right_wall == False and no_left_wall == True:
				movingBackward(vel_msg, velocity_publisher, t0, current_distance, distance[1], speed, -0.1, front)
				turnCCW(vel_msg, velocity_publisher, t0, 0, 45, 30)
			else:
				movingBackward(vel_msg, velocity_publisher, t0, current_distance, distance[1], speed, -0.1, front)
				turnCW(vel_msg, velocity_publisher, t0, 0, 45, 30) 
			
if __name__ == '__main__':
	escapeMaze()