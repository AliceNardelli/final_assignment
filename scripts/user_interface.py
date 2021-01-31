#! /usr/bin/env python

# import ros stuff
import rospy
from std_srvs.srv import *
import time

#sevice for menage the user interface of bug algorithm
#when the service is requested is asked to user of inserted a desired position
def set_new_pos(req):

    x = float(raw_input('x :'))
    y = float(raw_input('y :'))
    #take the start_time, moment when the robot start reach a new goal (manage that the goal is reacheble)
    start_time=time.clock()
    rospy.set_param("start_time", start_time)
    #set parameter user to one, this parameter is used for call the user_interface_assignment only when the target has been reached
    rospy.set_param("user", 1)
    #set the desired position inserted by user between the parameters
    rospy.set_param("des_pos_x", x)
    rospy.set_param("des_pos_y", y)
    print("Thanks! Let's reach the next position")

            
    return []


def main():
    #init node 
    rospy.init_node('user_interface')
    #init service
    srv = rospy.Service('user_interface', Empty, set_new_pos)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
