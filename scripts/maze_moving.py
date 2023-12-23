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
	vel_msg.angular.z = 0 #set angular z to zero  # Publish the velocity to stop the robot
	velocity_publisher.publish(vel_msg) 
	time.sleep(1) #stop for 1 second
def movingForward(vel_msg, velocity_publisher, t0, current_distance, distance, speed, forwardSpeed, front):
	vel_msg.linear.x = forwardSpeed  
	vel_msg.angular.z = 0 #initialize angular z to zero   
	print('Is moving')
	while(current_distance < distance):
		velocity_publisher.publish(vel_msg)#Publish the velocity		  #Take actual time to velocity calculation
		t1 = rospy.Time.now().to_sec()
		current_distance = speed*(t1-t0)#calculates distance#Stop the robot when the front distance from obstacle is smaller than
	if (front < 0.75):
		stop(vel_msg, velocity_publisher)
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
		t0 = rospy.Time.now().to_sec() #define the variable for current distance
		current_distance = 0 #read the input from lazer scan
		scan_msg = rospy.wait_for_message("scan", LaserScan)
		front = scan_msg.ranges[1]
		left = scan_msg.ranges[90]
		top_left = scan_msg.ranges[45]
		right = scan_msg.ranges[275]
		top_right = scan_msg.ranges[315] #direction is smaller than 2.0, else right wall is detected   
		   
		if (right< 0.35) or (top_right < 0.45):
			no_right_wall = False #False becuase right wall is detected   
			print('Right wall is detected')#display message
		else:
			no_right_wall = True #True becuase right wall is not detected		   
			print('Right wall is not detected') #display message
		if no_right_wall == True:
			if front < 0.65:
				print('Move backward')#display message
				movingBackward(vel_msg, velocity_publisher, t0, current_distance, distance[2], speed, -0.25, front)
				print('Turn clockwise because no right wall')
				turnCW(vel_msg, velocity_publisher, t0, 0, 45, 45) 
				print('Move forward because no right wall')		 
			if front < 0.75:
				#Move forward with distance of 0.1 when distance from front
				#wall is smaller than 0.5
				movingForward(vel_msg, velocity_publisher, t0, current_distance, distance[1], speed, 0.20, front)
			else:
				movingForward(vel_msg, velocity_publisher, t0, current_distance, distance[1], speed, 0.20, front)
		else:
			#No front wall if the distance between the robot and obstacles if 
			#the distance in front and top left   #direction is smaller than 1.0, else front wall is detected
			if (front > 0.65) or (top_left > 0.65):
				#True because no front wall is detected
				no_front_wall = True
				print('Front wall is not detected')#display message
			else:
				#False because front wall is detected
				no_front_wall = False
				print('Front wall is detected')#display message
		 
				#Move forward if there is no front wall
			if (no_front_wall == True) and (front > 0.55):
				if (front < 0.5):
					print('Move forward because no front wall')#display message
					#Move forward with distance of 0.1 when distance from front   
					#wall is smaller than 0.5
					movingForward(vel_msg, velocity_publisher, t0, current_distance,distance[1], speed, 0.20, front)
				else:
					#Move forward with distance of 0.1 when distance from front  
					#wall is greater than 0.5
					movingForward(vel_msg, velocity_publisher, t0,current_distance,distance[2], speed, 0.20, front)
						 
						  #Turn clockwise if neither front wall, left, and right wall are  
						  #detected. Indeed, turn counter-clockwise if both front wall and 
						  #right wall are detected
			else:
				if (left > 0.55) and ((right > 0.45) or (top_right > 0.65)):		 
					if front < 0.60:
						print('Move backward')#display message
						#move backward if the distance in front direction is smaller 
						#than 1.0
						movingForward(vel_msg, velocity_publisher, t0, current_distance,distance[0], speed, 0.20, front)
						movingBackward(vel_msg, velocity_publisher, t0,current_distance,distance[2], speed, -0.25, front)
						print('Turn clockwise because no right wall')#display message		
						#turn 90 degree clockwise
						turnCW(vel_msg, velocity_publisher, t0, 0, 45, 45)
						stop(vel_msg,velocity_publisher)
		   
				else:
					if front < 0.65:
						print('Move backward')#display message
						#move backward if the distance in front direction is smaller  
						#than 1.0
						movingForward(vel_msg, velocity_publisher, t0, current_distance, distance[0], speed, 0.20, front)
						movingBackward(vel_msg, velocity_publisher,t0,current_distance,distance[2], speed, -0.25, front)
						print('Turn counter-clockwise because have front wall')		   #turn 90 degree counter-clockwise
						time.sleep(1)
						turnCCW(vel_msg, velocity_publisher, t0, 0, 45, 45)# The node will stop when press control + C		
						stop(vel_msg,velocity_publisher)   
	rospy.spin()
if __name__ == '__main__':
	escapeMaze()
