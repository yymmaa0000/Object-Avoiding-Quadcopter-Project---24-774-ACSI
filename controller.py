import rospy
from geometry_msgs.msg import Twist, Pose
from optitrack.msg import RigidBodyArray
import tf
import math
import time

# linear x,y = [-30,30] 
# linear z = [0,60000]
# angular x,y = 0
# angular z = [-200,200]

# warning, please make sure that the yaw angle measured from optitrack system
# matches the yaw angle represented by the actual space!

PI = 3.1415927 

desired_location1 = [0.0,0.0,1.5] #x,y,z
desired_location2 = [-1.2,0.0,1.5] #x,y,z
# desired_location3 = [0.0,0.0,0.7] #x,y,z
desired_location = desired_location1

start = time.time()

prev_error = [0.0,0.0,0.0]
error_sum = [0.0,0.0,0.0]
Hover_Speed = 33000.0

Kp = [10.0,-10.0,30000.0]
Kd = [50.0,-50.0,40000.0]
Ki = [2.0,-2.0,400.0]
old_d = 0.0;

Skip_this_time = True

frequency = 50.0 #HZ
dt = 1.0 / frequency
rigidBodyIdx = 0
obstacleIdex = 1

def distance(x1,y1,z1,x2,y2,z2):
    temp = (x1-x2)**2+(y1-y2)**2+(z1-z2)**2
    return temp**0.5

def calculateXYerror(targetX,targetY,DroneX,DroneY,DroneYaw):
	dx = targetX - DroneX
	dy = targetY - DroneY
	y_error = -dx*math.sin(DroneYaw) + dy * math.cos(DroneYaw)
	x_error = dx*math.cos(DroneYaw) + dy * math.sin(DroneYaw)
	return x_error, y_error

def optitrackCallback(data):
	global pub, joyCommand, rate, rigidBodyIdx, dt, Hover_Speed
	global desired_location, prev_error, error_sum, Skip_this_time,old_d

	if Skip_this_time:
		Skip_this_time = False
		return
	else: Skip_this_time = True

	if (time.time()-start <= 2):
		joyCommand.linear.x = 0.0
		joyCommand.linear.y = 0.0
		joyCommand.linear.z = Hover_Speed * (time.time()-start )/2.0
		joyCommand.angular.x = 0.0
		joyCommand.angular.y = 0.0
		joyCommand.angular.z = 0.0
		pub.publish(joyCommand)
		return
	# elif (15 >= time.time()-start >= 10): 
	# 	desired_location  = desired_location2
	# elif (20 >= time.time()-start >= 15): 
	# 	desired_location  = desired_location3
	# elif (time.time()-start >= 25):
	# 	joyCommand.linear.x = 0.0
	# 	joyCommand.linear.y = 0.0
	# 	joyCommand.linear.z = 0.0
	# 	joyCommand.angular.x = 0.0
	# 	joyCommand.angular.y = 0.0
	# 	joyCommand.angular.z = 0.0
	# 	pub.publish(joyCommand)
	# 	return

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

	obsPose = data.bodies[obstacleIdex].pose
	obsPosition = obsPose.position
	obsOrientation = obsPose.orientation

	obsPos_x = obsPosition.x
	obsPos_y = obsPosition.y
	obsPos_z = obsPosition.z
	obsAng_x = obsOrientation.x
	obsAng_y = obsOrientation.y
	obsAng_z = obsOrientation.z
	obsAng_w = obsOrientation.w

	print("=======================================================================================")
	print("Drone Position:", cflPosition)
	print("Ball Position:", obsPose)
	# print("Orientation:", cflOrientation)
	eulerAngles = tf.transformations.euler_from_quaternion([cflAng_x, cflAng_y, cflAng_z, cflAng_w])
	# print("EulerAngles\n", eulerAngles[0]/PI*180, eulerAngles[1]/PI*180,eulerAngles[2]/PI*180)

	d = distance(obsPos_x,obsPos_y,obsPos_z,cflPos_x,cflPos_y,cflPos_z)
	d_diff = d - old_d
	print("SHABIIIIII", d_diff)
	if d < 2.0 or d_diff < -0.005: desired_location = desired_location2
	else: desired_location = desired_location1

	old_d = d

	joyCommand.linear.x = 0.0
	joyCommand.linear.y = 0.0
	joyCommand.linear.z = 50000.0
	joyCommand.angular.x = 0.0
	joyCommand.angular.y = 0.0
	joyCommand.angular.z = 0.0
	
	error = [0.0]*3
	error_dot = [0.0]*3
	error[0], error[1] = calculateXYerror(desired_location[0],desired_location[1],cflPos_x,cflPos_y,eulerAngles[2])
	error[2] = desired_location[2] - cflPos_z
	
	for i in range(3):
		error_dot[i] = (error[i] - prev_error[i])/dt
		error_sum[i] += error[i] * dt
		
	joyCommand.linear.x = Kp[0]*error[0] + Kd[0]*error_dot[0] + Ki[0]*error_sum[0]
	joyCommand.linear.y = Kp[1]*error[1] + Kd[1]*error_dot[1] + Ki[1]*error_sum[1]
	joyCommand.linear.z = Kp[2]*error[2] + Kd[2]*error_dot[2] + Ki[2]*error_sum[2]
	joyCommand.linear.z += Hover_Speed
	
	for i in range(3):
		prev_error[i] = error[i]
	
	joyCommand.linear.x = min(30,joyCommand.linear.x)
	joyCommand.linear.x = max(-30,joyCommand.linear.x)
	joyCommand.linear.y = min(30,joyCommand.linear.y)
	joyCommand.linear.y = max(-30,joyCommand.linear.y)
	joyCommand.linear.z = min(60000,joyCommand.linear.z)
	joyCommand.linear.z = max(Hover_Speed,joyCommand.linear.z)
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