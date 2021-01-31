Alice Nardelli 4528692

Content of the package:

In this code are present nine different nodes:

simulation.py  
This node is needed for manage the whole simulation and supports the structure of the entire code. It checks the state of the robot and activate on consequance the    needed services.

user_interface_assignment.py
This node is a ros service implemented for get from the user the state of the robot.

move_randomly_srv.py
It is a ros service node that implements the behaviour of going on in reaching different random target positions between six possible ones.

random_service.py
It is a ros service node that set a random position that robot will acquire.

wall_follow_service_m.py
It is a service for simulates the behaviour of wall following.

bug_m.py
This is a service for implementing bug algorithm behaviour.

user_interface.py
This is a service used by bug algorithm for getting the from user the target position that robot want to aquire.

go_to_point_service_m.py
This is a node service used by bug algorithm for implement go to point behaviour.

wall_follower_service_bug.py
This is a node service used by bug algorithm for avoiding to collides with wall while reach the target.

Note: even if the two nodes for wall follow behaviour have in common the code two different node are needed. As the fact that when robot simulates wall follow behaviour bug algorithm and rispective wall follow service are both disabled. Consequently two nodes are needed.

Messages published:

geometry_msgs/Twist() message on topic /cmd_vel is used to set a linear and angular velocity to the robot or for stopping robot.

move_base_msgs/MoveBaseActionGoal() message on topic /move_base/goal to publish a move base goal that robot has to reach.

actionlib_msgs/GoalID() message on topic /move_base/cancel to remove a target ones that is considered reached to avoid the overlap of different behaviours of robot.


Message subscribed:

nav_msgs/Odometry() message on topic /odom for getting actual position of the robot.

sensor_msgs/LaserScan() message on topic /scan for getting actual output of lasers.

geometry_msgs/Point() message to define position as point


Services:

eight different service are implemented (the description are inside the respective node):

/move_randomly in move_randomly_srv.py. Boolean service used to activated or not the behaviour.

/bug_algorithm_srv in bug_m.py. Boolean service used to activated or not the behaviour.

/wall_follower_switch_bug in wall_follower_service_bug.py. Boolean service used to activated or not the behaviour.

/wall_follower_switch in wall_follower_service_m.py. Boolean service used to activated or not the behaviour.

/go_to_point_switch in go_to_point_service_m.py. Boolean service used to activated or not the behaviour.

/random_service in random_service.py. Publish a new target position to reach.

/user_interface in user_interface.py. Ask for a target position to reach with bug algorithm and publish it.

/user_interface_assignment in user_interface_assignment.py. Service for change state of robot.

Parameters:

"des_pos_x" and "des_pos_y" are two parameters used to know the coordinates of desired position to reach.

"state" is a parameter for knowing the actual state of the robot

"change" is a boolean parameter for knowing when user iserted a new target for the robot. it is used to manage the change of status.

"start_time" is a parameter for knowing the instant in which robot begun to reach a certain point insert by user. It is used for implementing a timer to stop the bug algorithm behaviour if a certain position is not reacheable.

"user" is a parameter used by bug algorithm to call the /user_interface_assignment service only in some moments

Launch files:

simulation_gmapping.launch for gmapping package containing robot description, gazebo,rviz, and a node for slam implementation.
move_base.launch for move_base planner
bug0.launch for bug algorithm planner
assignment.launch to launch the simulation which include bug0.launch (node and parameter have been previously presented)

To execute the code is needed execute in three different shells:
roslaunch final_assignment simulation_gmapping.launch
roslaunch final_assignment move_base.launch
roslaunch final_assignment assignment.launch

The sofware architecture for control the robot in the environment is organized in such a way that the behaviour of the robot are chosen by user. There are five possible behaviours:
1 Goes on in reaching random position choosen between (-4,-3) (-4,2) (-4,7) (5,-7) (5,-3) (5,1)
2 Get a position inserted by the user between the six possible
3 Robot start to move in the environment follow the wall
4 Robot stop at the current position
5 Robot pass to bug algorithm behaviour

When the state of the robot is changed the user interface ask for a new target so the user can suddently insert a new one. If the robot state is 1 or 2 it before aquires the goal then change status.
If the state is bug algorithm a time out has been implemented in such a way that if an inserted goal is not reacheable the user interface ask for a new target. It is possible to exit from bug algorithm behaviour if the robot reaches the desired position or if time exipires.

Software architecture:

gmapping packages is used for simulate the robot and its interaction with the environment. This simulates the module devoted for controller plugins, sensor plugins, and simulated hardware. 
It get as input /cmd_vel messages whereas as otput publishes message of type /odom /scan.
Moreover Slam (simultaneous localization and mapping) implementation is done by Gmapping.
Implemented nodes are implemented for control the behaviour of the robot. The overall stucture is managed by the node simulation.py then for each behaviour is implemented both collision avoidance and path plannig. (according to what target user has chosen). 
In behavour 1 and 2 is used move_base planner. Move_base is a action: publishing a message on /move_base/goal the robot will attempt to reach it according to a local planner and a global planner. Local planner get informations from sensor (laser) and path plan in limited space. Global planner on the other hand builds a global map and considers all possible informations ever received for reaching the goal. It decides for a path according to Dijkstra's algorithm for get the desired position. Two cost maps one global and one local are built in run time for collision avoidance. In this case also message of type /move_base/cancel are used for managing the change of behaviour.
In behaviour 3 path plannig and collision avoidance are done by subscribing from /scan and publishing on /cmd_vel.
In bug algorithm behaviour path planning is implemented by service go to point whereas collision avoidance is implemented by wall follow service. At this purpose it reads message of type /odom and /scan and publish on /cmd_vel.
Finally two different nodes and some pieces of code have been implemented for user interface. User can change the status of the robot and is advised about status of the robot when status really change. Moreover user is warned about the goal position of move_base, when a goal is reached and if robot is stopped the position where it is.

System limitation's and possible improvements:

A limit of this project is the behaviour get a target with move_base randomly chosen or chosen by user, as the fact that for our project the goal is reached only when distance from goal is smaller then 0.5, in reality the goal is not reached, this makes it difficult to control the change of status if the use insert another target at the same time. This gap of this control architecture is visible in the change of status from 1 to 3. A possible improvement is publishing a message on /move_base/cancel. Another possible improvement that I've done is when a goal is reached before publishing another /move_base/goal I check the status. If status has been changed by user when robot was reaching the goal system doesn't publish a new goal. Anyway it sometimes doesn't work and robot has misunderstanding between target 1 and 3. It's status is wall following, it follows wall, but a move_base remains published. Maybe a better synchronisation using time.sleep(..) should be a way. 

Planning with bug algorithm has big limits: as the fact that in the environment has lot of wall and the precision is quite hight so suddenly robot never reach the goal but goes on in wall follow. Using another planner to reach a random position such as move_base is certainly a better choice.

Due that the map is built in runtime and the environment has a lot of rooms when the robot is reaching a certain position the path chose from Dijkstra's algorithm bring the robot in another room where there is no the goal. When the robot understand through sensors that between it and the goal there is a wall the optimal path is changed. This type of planning makes always reach the goal if it is reacheable. Maybe the use of a preexisting map and AMCL could be a better choice for localization.

The timer of the bug algorithm sometimes didn't work. The start time is get when the new goal position. More over to be sure start time is update each time the service is called to avoid that if it pass to much time before activate the service the time elapsed became too great. The actual time is get at actual instant if service is active. Sometimes timer say that time is up and goal is not reacheable even if the time elepsed in reality is certainly smaller than 120s. Sometimes the time effectively elapsed is greater than 120s but the timer didn't advise user. I didn't find the error in implementation of the timer. Maybe there is some delay inside the loop but I didn't find where. 
To simulate the whole architecture without activate bug algorithm or not insert 5 as target or in simulation.py argument line 158 and disargument line 161.


									    
