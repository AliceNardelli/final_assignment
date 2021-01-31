#! /usr/bin/env python

# import ros stuff
import rospy
import random
import time
from std_srvs.srv import *


#this service has been implemented for managing the user interface 

def set_goal(req):
    #when this service is requested it ask to user to insert a new target
    print("insert a target: ")
    print("1: move randomly")
    print("2: insert a target")
    print("3: fallowing wall")
    print("4: stop")
    print("5: pass to bug algorithm")
    target = int(raw_input('target :'))
    #check the input
    #if target is valid put the inserted value on parameter state
    #put parameter chage to 1 to state that the target has been changed
    if target==1 or target==2 or target==3 or target==4 or target==5:
	    rospy.set_param("state", target)
	    rospy.set_param("change", 1)
    #if input is not valid stop the robot then insert a new target
    else:
           print('not valid input')
           print('robot will be stopped')
           print('please insert a valid target')
    	   rospy.set_param("state", 4)
	   rospy.set_param("change", 1)
    return []


def main():
    #new node
    rospy.init_node('user_interface_assignment')
    #user interface service
    srv = rospy.Service('user_interface_assignment', Empty, set_goal)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
