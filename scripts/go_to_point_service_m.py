#! /usr/bin/env python
#this is a node for the behaviour go to point of the bug algorithm
# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

active_ = False

# robot state variables
position_ = Point()
yaw_ = 0
# machine state
state_ = 0
# goal
desired_position_ = Point()
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = 0
# parameters
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.3

kp_a = 3.0  # In ROS Noetic, it may be necessary to change the sign of this proportional controller
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

# publishers
pub = None

# service callbacks


def go_to_point_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

# callbacks

#get actual position and orentiation of the robot
def clbk_odom(msg):
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def change_state(state):
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)


def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

#fix the orientation of the robot
def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_2_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    #get the error angle orientation
    rospy.loginfo(err_yaw)
    #publish a message Twist() with an angular velocity
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a

    pub.publish(twist_msg)

    # state change conditions
    #if error is smaller of precision robot goes straight
    if math.fabs(err_yaw) <= yaw_precision_2_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(1)

#robot go straight
def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    #compute error in terms of both distance and orientation
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    #if distance is bigger then precision a message Twist() in linear velocity is published for making the robot go straight
    #an angular velocity proportional to orientation angle is published too
    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = kp_d*(err_pos)
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        pub.publish(twist_msg)
    else:
        #if distance is smaller then precision robot reached the target, it is stopped
        print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    #if error orientation is bigger then precision the robot change state in such a way to adjust precision
    if math.fabs(err_yaw) > yaw_precision_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(0)

#robot stop publishing a massage Twist() with zero velocity
def done(des_pos):
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)
    #error is computed if it is bigger of 0.35 it changes state and goes on in going to point
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    if(err_pos > 0.35):
        change_state(0)


def main():
    global pub, active_, desired_position_
    #init node
    rospy.init_node('go_to_point')
    #publisher for topic /cmd_vel  for setting velocity of the robot
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    #subscriber of the odometry of the robot
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    #service for the behaviour go to point
    srv = rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        desired_position_.x = rospy.get_param('des_pos_x')
        desired_position_.y = rospy.get_param('des_pos_y')
        if not active_:
            continue
        else:
            #check which is state of robot and react on consequance
            if state_ == 0:
                #fix the oriantation of the robot 
                fix_yaw(desired_position_)
            elif state_ == 1:
                #make robot go straight
                go_straight_ahead(desired_position_)
            elif state_ == 2:
                #stop robot
                done(desired_position_)
            else:
                rospy.logerr('Unknown state!')

        rate.sleep()


if __name__ == '__main__':
    main()
