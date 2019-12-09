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

default_reference = [0.0,0.0,0.7]#x,y,z
desired_location = default_reference

desired_location2 = [-1.0,1.0,1.5] #x,y,z
desired_location3 = [0.0,1.3,0.3] #x,y,z

start_time = time.time()
last_time = time.time()

Skip_this_time = True

frequency = 50.0 #HZ
dt = 1.0 / frequency

rigidBodyIdx_drone = 0
rigidBodyIdx_obstacle = 1
old_d = 0

iii = 0

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
	global pub, joyCommand, rate, Skip_this_time,desired_location, last_time, iii, old_d

	if Skip_this_time:
		Skip_this_time = False
		return
	else: Skip_this_time = True

	# curr_time = time.time()
	# if (curr_time-last_time) < dt: return
	# last_time = curr_time


	# if (time_passed() <= 4): 
	# 	take_off()
	# 	return



	# elif (15 >= time_passed() >= 10):  desired_location  = desired_location2
	# elif (20 >= time_passed() >= 15):  desired_location  = desired_location3
	# elif (time_passed() >= 25): 
	# 	land()
	# 	return
		
	cflPose_drone = data.bodies[rigidBodyIdx_drone].pose
	# cflPose_drone = data.bodies[2].pose
	cflPose_obstacle = data.bodies[rigidBodyIdx_obstacle].pose
	
	print("=======================================================================================")
	# if iii < 0: iii += 1
	# else: 
	# 	# print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
	# 	iii = 0
	# 	desired_location = obstacle_avoidance_planner2.plan(cflPose_drone,cflPose_obstacle,default_reference)
	# 	# desired_location = simple_planner.plan(cflPose_drone,cflPose_obstacle,default_reference)
	drone_location = utility_function.GetPositionInfo(cflPose_drone)
	drone_x = drone_location[0]
	drone_y = drone_location[1]
	drone_z = drone_location[2]
	
	ball_location = utility_function.GetPositionInfo(cflPose_obstacle)
	ball_x = ball_location[0]
	ball_y = ball_location[1]
	ball_z = ball_location[2]

	d = obstacle_avoidance_planner2.distance(drone_x,drone_y,drone_z,ball_x,ball_y,ball_z)
	d_diff = d - old_d
	old_d = d

	if d < 2.0 or d_diff < -0.005: desired_location = [-1.2,0.0,0.7]
	else: desired_location =  default_reference

	print("Reference location: ",desired_location[0],desired_location[1],desired_location[2])
	controller_output = Position_controller.ControlOutput(desired_location,cflPose_drone,dt)
	
	joyCommand.linear.x = controller_output[0]
	joyCommand.linear.y = controller_output[1]
	joyCommand.linear.z = controller_output[2]
	joyCommand.angular.x = controller_output[3]
	joyCommand.angular.y = controller_output[4]
	joyCommand.angular.z = controller_output[5]
	
	pub.publish(joyCommand)
	# rospy.loginfo("sent command")


rospy.init_node("control_crazyflie")
pub = rospy.Publisher("/crazyflie/cmd_vel", Twist, queue_size=10)
joyCommand = Twist()
rate = rospy.Rate(frequency)
rospy.Subscriber("/optitrack/rigid_bodies", RigidBodyArray, optitrackCallback)

take_off()
rospy.spin()