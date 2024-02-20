#!/usr/bin/env python3

import rospy
from maze_project.srv import SetVelocity

def set_velocity_client(linear_speed, angular_speed):
    rospy.wait_for_service('set_velocity')
    try:
        set_velocity = rospy.ServiceProxy('set_velocity', SetVelocity)
        response = set_velocity(linear_speed, angular_speed)
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return False

if __name__ == "__main__":
    rospy.init_node('set_velocity_client')
    linear_speed = float(input("Enter linear speed: "))
    angular_speed = float(input("Enter angular speed: "))
    success = set_velocity_client(linear_speed, angular_speed)
    if success:
        rospy.loginfo("Velocity set successfully")
    else:
        rospy.logerr("Failed to set velocity")
