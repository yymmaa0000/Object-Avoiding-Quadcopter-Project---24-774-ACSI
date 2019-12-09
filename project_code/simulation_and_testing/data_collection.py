"""
24-774 Obstacle Avoidance Drone Project
obstacle_avoidance_planner for dodging moving obstacles
Created on Sun Nov 10 15:13:30 2019
author: XingYu Wang
"""
import rospy
from geometry_msgs.msg import Twist, Pose
from optitrack.msg import RigidBodyArray
import time
import Position_controller
import obstacle_avoidance_planner
import obstacle_avoidance_planner2
import simple_planner
import utility_function

# linear x,y = [-30,30] 
# linear z = [0,60000]
# angular x,y = 0
# angular z = [-200,200]

# warning, please make sure that the yaw angle measured from optitrack system
# matches the yaw angle represented by the actual space!

default_reference = [0.0,0.0,1.5]#x,y,z
desired_location = default_reference

# desired_location2 = [-1.0,1.0,1.5] #x,y,z
# desired_location3 = [0.0,1.3,0.3] #x,y,z

prev_ball_location = [0.0,0.0,0.0]
last_reference = desired_location

start_time = time.time()
last_time = time.time()

Skip_this_time = True

frequency = 50.0 #HZ
dt = 1.0 / frequency

rigidBodyIdx_drone = 0
rigidBodyIdx_obstacle = 1
old_d = 0

def time_passed():
	return time.time()-start_time

def take_off():
	global pub, joyCommand
	Hover_Speed = 33000.0
	take_off_time = 4.0
	start = time.time()

	while (time.time() - start < take_off_time):
		joyCommand.linear.x = 0.0
		joyCommand.linear.y = 0.0
		joyCommand.linear.z = Hover_Speed * (time.time()-start)/take_off_time
		joyCommand.angular.x = 0.0
		joyCommand.angular.y = 0.0
		joyCommand.angular.z = 0.0
		pub.publish(joyCommand)

def land():
	global pub, joyCommand
	joyCommand.linear.x = 0.0
	joyCommand.linear.y = 0.0
	joyCommand.linear.z = 0.0
	joyCommand.angular.x = 0.0
	joyCommand.angular.y = 0.0
	joyCommand.angular.z = 0.0
	pub.publish(joyCommand)


def optitrackCallback(data):
	global pub, joyCommand, rate, Skip_this_time,desired_location, last_time, old_d,prev_ball_location

	if Skip_this_time:
		Skip_this_time = False
		return
	else: Skip_this_time = True

	cflPose_obstacle = data.bodies[rigidBodyIdx_obstacle].pose
	ball_location = utility_function.GetPositionInfo(cflPose_obstacle)
	ball_x = ball_location[0]
	ball_y = ball_location[1]
	ball_z = ball_location[2]
	print ball_x,ball_y,ball_z 
	
	
	joyCommand.linear.x = 0
	joyCommand.linear.y = 0
	joyCommand.linear.z = 0
	joyCommand.angular.x = 0
	joyCommand.angular.y = 0
	joyCommand.angular.z = 0
	
	pub.publish(joyCommand)
	# rospy.loginfo("sent command")


rospy.init_node("control_crazyflie")
pub = rospy.Publisher("/crazyflie/cmd_vel", Twist, queue_size=10)
joyCommand = Twist()
rate = rospy.Rate(frequency)
rospy.Subscriber("/optitrack/rigid_bodies", RigidBodyArray, optitrackCallback)

take_off()
rospy.spin()