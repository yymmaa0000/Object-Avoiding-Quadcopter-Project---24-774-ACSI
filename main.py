import rospy
from geometry_msgs.msg import Twist, Pose
from optitrack.msg import RigidBodyArray
import time
import Position_controller

# linear x,y = [-30,30] 
# linear z = [0,60000]
# angular x,y = 0
# angular z = [-200,200]

# warning, please make sure that the yaw angle measured from optitrack system
# matches the yaw angle represented by the actual space!

desired_location = [0.0,0.0,0.7] #x,y,z
desired_location2 = [-1.0,1.0,1.5] #x,y,z
desired_location3 = [0.0,1.3,0.3] #x,y,z

start_time = time.time()

Skip_this_time = True

frequency = 50.0 #HZ
dt = 1.0 / frequency

rigidBodyIdx_drone = 0

def time_passed():
	return time.time()-start_time

def take_off():
	global pub, joyCommand
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
	global pub, joyCommand, rate

	if Skip_this_time:
		Skip_this_time = False
		return
	else: Skip_this_time = True

	if (time.time()-start <= 2): 
		take_off()
		return
	elif (15 >= time.time()-start >= 10):  desired_location  = desired_location2
	elif (20 >= time.time()-start >= 15):  desired_location  = desired_location3
	elif (time.time()-start >= 25): 
		land()
		return
		
	cflPose_drone = data.bodies[rigidBodyIdx_drone].pose
	controller_output = Position_controller.ControlOutput(desired_location,cflPose_drone,dt)
	
	joyCommand.linear.x = controller_output[0]
	joyCommand.linear.y = controller_output[1]
	joyCommand.linear.z = controller_output[2]
	joyCommand.angular.x = controller_output[3]
	joyCommand.angular.y = controller_output[4]
	joyCommand.angular.z = controller_output[5]
	pub.publish(joyCommand)
	rospy.loginfo("sent command")


rospy.init_node("control_crazyflie")
pub = rospy.Publisher("/crazyflie/cmd_vel", Twist, queue_size=10)
joyCommand = Twist()
rate = rospy.Rate(frequency)
rospy.Subscriber("/optitrack/rigid_bodies", RigidBodyArray, optitrackCallback)


rospy.spin()