#! /usr/bin/env python

import rospy
import time

# import ros message
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations

# import ros service
from std_srvs.srv import *
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID

import math
#global needed variables
position_ = Point()
pub = None
pub2=None
srv_client_get_random_ = None
active_=False
counter2=True;


#callback of subscribers for getting the actual position
def clbk_odom(msg):
    global position_

    # position
    position_ = msg.pose.pose.position
    



#service
def move_randomly(req):
    global active_,counter2
    #check the request of service 
    #consequently activate or not the behaviour 
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    counter2=True
           
    return res

def pub_goal():
        #publish a MoveBaseActionGoal() message on topic /move_base/goal 
        msg_goal=MoveBaseActionGoal()
	msg_goal.goal.target_pose.header.frame_id="map";
	msg_goal.goal.target_pose.pose.orientation.w=1;
        #take from parameters the position that must be reached
	msg_goal.goal.target_pose.pose.position.x=rospy.get_param('des_pos_x');
	msg_goal.goal.target_pose.pose.position.y=rospy.get_param('des_pos_y');
	pub.publish(msg_goal)
        
        print("\n goal: x=" + str(rospy.get_param('des_pos_x')) + " y=" + str(rospy.get_param('des_pos_y')))
        return []

#this function return the distance between the actual position of the robot and the desired one 
def distance():
    global position_
    d=math.sqrt(pow(position_.y-rospy.get_param('des_pos_y'), 2) +
                       pow(position_.x-rospy.get_param('des_pos_x'), 2))
   
    return d


def main():
    global position_,srv_client_get_random_,pub,active_,pub2,counter2,change,state_
    
    #varibles counter2 is used to publish a random position when service is activated
    counter2=True
    #init a new node
    rospy.init_node('move_randomly')
    #service for move randomly behaviour
    srv = rospy.Service('move_randomly', SetBool, move_randomly)
    #service client for getting a random position
    srv_client_get_random_ = rospy.ServiceProxy('/random_service', Empty)

    #publisher on /move_base/goal for reaching a new position
    pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)
    #publisher on /move_base/cancel for cancel the goal once that is reached
    pub2 = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
    #subscriber of topic /odom for getting actual position
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
            #if service is active
	    if active_:
                #if the service has been just activate ask for a random position and publish on /move_base
		if counter2:
                       
		        resp = srv_client_get_random_()
		        pub_goal()
		        counter2=False
          
		msg_cancel=GoalID()
		a=1
		a=distance()
                #take the value of distance from robot to goal
                #when distance is smaller then 0.5 the goal is reached
		if a<0.5 :

                        print('GOAL REACHED')
                        change=rospy.get_param("change")
                        state_=rospy.get_param("state")
                        
                        #if the status of the robot has not been changed when robot was reaching the target publish a new random target 
                        if change==0:
                                
                        
				pub2.publish(msg_cancel)
		                
				resp = srv_client_get_random_()
				pub_goal()
                        #if the status has not been changed by robot disable this behaviour
                        else:
                               
                               pub2.publish(msg_cancel)
                               active_=False
                               
            else:
                    counter2=True  
	    rate.sleep()





    
   
      
      


if __name__ == '__main__':
    main()
