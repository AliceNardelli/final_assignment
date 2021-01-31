#! /usr/bin/env python

# import ros stuff
import rospy
import random
from std_srvs.srv import *
#this service is used for set a new goal position randomly chosen between the 6 possible

def random_service(req):
    #extract a random number from 1 to 6
    #the six possible positions has been enumerated from 1 to 6
    #set the parameters of desired position according to the number that has been extracted
    number=random.randint(1,6)
    if number == 1:
         rospy.set_param("des_pos_x", -4)
         rospy.set_param("des_pos_y", -3)
    if number == 2:
         rospy.set_param("des_pos_x", -4)
         rospy.set_param("des_pos_y", 2)
    if number == 3:
         rospy.set_param("des_pos_x", -4)
         rospy.set_param("des_pos_y", 7)
    if number == 4:
         rospy.set_param("des_pos_x", 5)
         rospy.set_param("des_pos_y", -7)
    if number == 5:
         rospy.set_param("des_pos_x", 5)
         rospy.set_param("des_pos_y", -3)
    if number == 6:
         rospy.set_param("des_pos_x", 5)
         rospy.set_param("des_pos_y", 1)
    
    return []


def main():
    #init node
    rospy.init_node('random_service')
    #init service
    srv = rospy.Service('random_service', Empty, random_service)
    
    
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
       
        rate.sleep()


if __name__ == '__main__':
    main()
