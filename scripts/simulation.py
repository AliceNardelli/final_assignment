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

#need of public publishers and service client
pub = None
pub2=None
pub3=None
srv_client_move_random_ = None
srv_client_wall_follower_ = None
srv_client_user_interface_assignment_=None
srv_client_bug_algorithm_ = None

#point for acutual position of the robot
position_ = Point()

regions_ = None
#variables
counter=True
change=0
#state of the robot
state_desc_ = ['random target', 'get target', 'wall fallowing','stop','bug']
state_ = rospy.get_param('state')
previous_state_=4
# callbacks 
#getting position
def clbk_odom(msg):
    global position_

    # position
    position_ = msg.pose.pose.position





#function for behaviour 2
def set_new_pos():
    global srv_client_user_interface_assignment_,previous_state_
    #ask to user to insert the input
    print("Insert a new position between (-4,-3) (-4,2) (-4,7) (5,-7) (5,-3) (5,1)")
    x = float(raw_input('x :'))
    y = float(raw_input('y :'))
   
    
    #check the input
    if ((x==-4 and (y==2 or y==-3 or y==7)) or (x==5 and (y==1 or y==-3 or y==-7) )):
            #if input is valid
            #set the input values in parameters for desired position
	    rospy.set_param("des_pos_x", x)
	    rospy.set_param("des_pos_y", y)
            #public a message MovebaseActionGoal() in topic /move_base/goal
	    msg_goal=MoveBaseActionGoal()
	    msg_goal.goal.target_pose.header.frame_id="map";
	    msg_goal.goal.target_pose.pose.orientation.w=1;
            #set in the position the desired ones
	    msg_goal.goal.target_pose.pose.position.x=rospy.get_param('des_pos_x');
	    msg_goal.goal.target_pose.pose.position.y=rospy.get_param('des_pos_y');
	    pub2.publish(msg_goal)
	    print("goal: x=" + str(rospy.get_param('des_pos_x')) + " y=" + str(rospy.get_param('des_pos_y')))
            resp = srv_client_user_interface_assignment_()
    else:
           #if input is not valid tell it to user 
           print("NOT VALID INPUT")         
           print('please insert another position')
           check_status()
    return []



#compute the distance from the robot actual position to the desired position
def distance():
    d=math.sqrt(pow(rospy.get_param('des_pos_y') - position_.y, 2) +
                        pow(rospy.get_param('des_pos_x')- position_.x, 2))
    return d

   

#when a new target is inserted by user this function manage the change of status
def check_status():
    
    global state_, state_desc_, position_,change
    global srv_client_move_random_ ,srv_client_wall_follower_,srv_client_user_interface_assignment_,srv_client_bug_algorithm_ 
    #here the status will be checked so parameter for change status is set to zero
    rospy.set_param("change",0)
    #get the new state of the robot
    state_ = rospy.get_param("state")
    log = "STATE: %s" % state_desc_[state_-1]
    rospy.loginfo(log)

    if state_ == 1:
        #set only the request for service for first behaviour to True 
        #set wall follow and bug algorithm behaviour to false
        #call the user interface for inserting a new target
        resp = srv_client_wall_follower_(False)
        resp = srv_client_move_random_(True)
        resp=srv_client_bug_algorithm_(False)
        resp = srv_client_user_interface_assignment_()


    if state_ == 2:
         
        #set get random position, wall follow and bug  algorithm services to false
        #call the function set_new_pos() to true for make inserting a new position by user
        #call the user interface for inserting a new target
        resp = srv_client_wall_follower_(False)
        resp = srv_client_move_random_(False)
        resp=srv_client_bug_algorithm_(False)
       
        set_new_pos()
        

    if state_ == 3:
        #set only the request for service for second behaviour to True 
        #set random target and bug algorithm behaviour to false
        #call the user interface for inserting a new target
        resp = srv_client_wall_follower_(True)
        resp = srv_client_move_random_(False) 
        resp=srv_client_bug_algorithm_(False)
        resp = srv_client_user_interface_assignment_()
        

    if state_ == 4:
       
        #set random target, bug algorithm and wall follow behaviour to false
        #publish a Twist() message on topic /cmd_vel for stopping the robot
        #call the user interface for inserting a new target
        resp = srv_client_wall_follower_(False) 
        resp = srv_client_move_random_(False)
        resp=srv_client_bug_algorithm_(False)
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.linear.y = 0
        twist_msg.angular.z = 0
        pub.publish(twist_msg)
        print("robot stopped at position: x= "+ str(position_.x) +" y=" + str(position_.y))
       
        resp = srv_client_user_interface_assignment_()

    if state_ == 5:
        #set only the request for service for bug algorithm behaviour to True 
        #set random target and wallfollow behaviour to false
        #To not activate never bug algorithm argument next line and dis-argoment the user intaerface service
        resp=srv_client_bug_algorithm_(True)
        resp = srv_client_wall_follower_(False) 
        resp = srv_client_move_random_(False)
        #resp = srv_client_user_interface_assignment_()

def main():

    time.sleep(2)

    global position_, state_,previous_state_,change,counter
    global srv_client_move_random_, srv_client_wall_follower_, srv_client_user_interface_assignment_,srv_client_bug_algorithm_ 
    global pub,pub2,pub3
    #init the node
    rospy.init_node('simulation')

    #subscriber of /odom for getting position
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    #publisher of /cmd_vel for stopping the robot
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    #publisher of /move_base/goal for setting a new goal position on move_base
    pub2 = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)
    #publisher of /move_base/cancel for removing the goal position on move_base
    pub3 = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)

    #service client for user interface
    srv_client_user_interface_assignment_ = rospy.ServiceProxy(
        '/user_interface_assignment', Empty)
    #service client for wall follower behaviour
    srv_client_wall_follower_ = rospy.ServiceProxy(
        '/wall_follower_switch', SetBool)
    #service client for move randomly behaviour
    srv_client_move_random_ = rospy.ServiceProxy('/move_randomly', SetBool)
    #service client for bug algorithm
    srv_client_bug_algorithm_ = rospy.ServiceProxy('/bug_algorithm_srv', SetBool)

    # initialize stop
    print("STARTING SIMULATION..")
    
    rospy.set_param("state", 4)
    #parameter change is set to 1 for check position
    rospy.set_param("change",1)
    previous_state_=rospy.get_param("state")
    check_status()
    
    
    rate = rospy.Rate(20)
    
    while not rospy.is_shutdown():
        #get needed parameters
        change=rospy.get_param("change")
        state_=rospy.get_param("state")
        #if a new target has been inserted before manage the stastus check the previous state
        #if previous state is 1 o 2 wait before the target has been reached then manage the change of status
        if change==1:

            if previous_state_==1:
                a=1
		a=distance()
                #go on in checking the distance only when distance is smaller then 0.5 cancel the move base goal and check the status
                if a<0.5:
                     msg_cancel=GoalID()
                     pub3.publish(msg_cancel)    
                     check_status()
                     previous_state_=state_

            elif previous_state_==2:
                a=1
		a=distance()
                #go on in checking the distance only when distance is smaller then 0.5 cancel the move base goal and check the status
                if a<0.5:
                     msg_cancel=GoalID()
                     pub3.publish(msg_cancel)
                     print('GOAL REACHED')
                     
                     check_status()
            
                     previous_state_=state_  
            #if previous state is different to 1 and to 2 check directly the new status 
	    else: 
                     msg_cancel=GoalID()
                     pub3.publish(msg_cancel)
                     check_status()
                     previous_state_=state_  
                     active_=False

        else:
              #if no change of status
              continue
        rate.sleep()


if __name__ == "__main__":
    main()
