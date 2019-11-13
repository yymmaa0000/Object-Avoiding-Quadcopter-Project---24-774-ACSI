import rospy
from geometry_msgs.msg import Twist, Pose
from optitrack.msg import RigidBodyArray
import tf
import math

# linear x,y = [-30,30] 
# linear z = [0,60000]
# angular x,y = 0
# angular z = [-200,200]

# warning, please make sure that the yaw angle measured from optitrack system
# matches the yaw angle represented by the actual space!

PI = 3.1415927 

desired_location = [0,0,2.0] #x,y,z
prev_error = [0,0,0]
error_sum = [0,0,0]

Kp = [0,0,60000.0]
Kd = [0,0,0]
Ki = [0,0,0]

frequency = 10 #HZ
dt = 1 / frequency
rigidBodyIdx = 0

def calculateXYerror(targetX,targetY,DroneX,DroneY,DroneYaw):
	dx = targetX - DroneX
	dy = targetY - DroneY
	x_error = dx*math.sin(DroneYaw) - dy * math.cos(DroneYaw)
	y_error = dx*math.cos(DroneYaw) + dy * math.sin(DroneYaw)
	return x_error, y_error

def optitrackCallback(data):
	global pub, joyCommand, rate, rigidBodyIdx, dt
	global desired_location, prev_error , error_sum

	cflPose = data.bodies[rigidBodyIdx].pose
	cflPosition = cflPose.position
	cflOrientation = cflPose.orientation

	cflPos_x = cflPosition.x
	cflPos_y = cflPosition.y
	cflPos_z = cflPosition.z
	cflAng_x = cflOrientation.x
	cflAng_y = cflOrientation.y
	cflAng_z = cflOrientation.z
	cflAng_w = cflOrientation.w

	print("Position:", cflPosition)
	# print("Orientation:", cflOrientation)
	eulerAngles = tf.transformations.euler_from_quaternion([cflAng_x, cflAng_y, cflAng_z, cflAng_w])
	print("EulerAngles\n", eulerAngles[0]/PI*180, eulerAngles[1]/PI*180,eulerAngles[2]/PI*180)

	joyCommand.linear.x = 0.0
	joyCommand.linear.y = 0.0
	joyCommand.linear.z = 50000.0
	joyCommand.angular.x = 0.0
	joyCommand.angular.y = 0.0
	joyCommand.angular.z = 0.0
	
	error = [0]*3
	error_dot = [0]*3
	error[0], error[1] = calculateXYerror(desired_location[0],desired_location[1],cflPos_x,cflPos_y,eulerAngles[2])
	error[2] = desired_location[2] - cflPos_z
	for i in range(3):
		error_dot[i] = (error[i] - prev_error[i])/dt
		error_sum[i] += error[i] * dt
	
	joyCommand.linear.x = Kp[0]*error[0] + Kd[0]*error_dot[0] + Ki[0]*error_sum[0]
	joyCommand.linear.y = Kp[1]*error[1] + Kd[1]*error_dot[1] + Ki[1]*error_sum[1]
	joyCommand.linear.z = Kp[2]*error[2] + Kd[2]*error_dot[2] + Ki[2]*error_sum[2]
	
	for i in range(3):
		prev_error[i] = error[i]
	
	joyCommand.linear.x = min(30,joyCommand.linear.x)
	joyCommand.linear.x = max(-30,joyCommand.linear.x)
	joyCommand.linear.y = min(30,joyCommand.linear.y)
	joyCommand.linear.y = max(-30,joyCommand.linear.y)
	joyCommand.linear.z = min(60000,joyCommand.linear.z)
	joyCommand.linear.z = max(33000,joyCommand.linear.z)
	print("Error X: ",error[0]," Error Y: ",error[1]," Error Z: ",error[2])
	print("Joy command X: ",joyCommand.linear.x," Joy command Y: ",joyCommand.linear.y," Joy command Z: ",joyCommand.linear.z)

	pub.publish(joyCommand)
	rospy.loginfo("sent command")


rospy.init_node("control_crazyflie")
pub = rospy.Publisher("/crazyflie/cmd_vel", Twist, queue_size=10)
joyCommand = Twist()
rate = rospy.Rate(frequency)
rospy.Subscriber("/optitrack/rigid_bodies", RigidBodyArray, optitrackCallback)


rospy.spin()