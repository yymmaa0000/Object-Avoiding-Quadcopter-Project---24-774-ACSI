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
import simple_planner

# linear x,y = [-30,30] 
# linear z = [0,60000]
# angular x,y = 0
# angular z = [-200,200]

# warning, please make sure that the yaw angle measured from optitrack system
# matches the yaw angle represented by the actual space!

default_reference = [0.0,0.0,0.4]#x,y,z
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

def time_passed():
	return time.time()-start_time

def take_off():
	global pub, joyCommand
	Hover_Speed = 33000.0
	take_off_time = 2.0
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
	global pub, joyCommand, rate, Skip_this_time,desired_location, last_time

	if Skip_this_time:
		Skip_this_time = False
		return
	else: Skip_this_time = True

	curr_time = time.time()
	if (curr_time-last_time) < dt: return
	last_time = curr_time


	if (time_passed() <= 2): 
		take_off()
		return

	# elif (15 >= time_passed() >= 10):  desired_location  = desired_location2
	# elif (20 >= time_passed() >= 15):  desired_location  = desired_location3
	# elif (time_passed() >= 25): 
	# 	land()
	# 	return
		
	cflPose_drone = data.bodies[rigidBodyIdx_drone].pose
	# cflPose_obstacle = data.bodies[rigidBodyIdx_obstacle].pose
	
	print("=======================================================================================")
	# desired_location = obstacle_avoidance_planner.plan(cflPose_drone,cflPose_obstacle,default_reference)
	# desired_location = simple_planner.plan(cflPose_drone,cflPose_obstacle,default_reference)
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


rospy.spin()