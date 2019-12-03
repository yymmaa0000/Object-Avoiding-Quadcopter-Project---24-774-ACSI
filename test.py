import rospy
from geometry_msgs.msg import Twist, Pose
from optitrack.msg import RigidBodyArray
import tf

# linear x,y = [-30,30] 
# linear z = [0,60000]
# angular x,y = 0
# angular z = [-200,200]

PI = 3.1415927 

desired_location = [0,0,2.0] #x,y,z
prev_error = [0,0,0]
error_sum = [0,0,0]

Kp = [0,0,60000.0]
Kd = [0,0,0]
Ki = [0,0,0]

def optitrackCallback(data):
	global pub, joyCommand, rate, rigidBodyIdx
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

	error_z = (desired_height - cflPos_z)
	error_dot_z = error_z - prev_error_z
	error_sum_z += error_z

	joyCommand.linear.z = Kpz*error_z + Kdz*error_dot_z + Kiz*error_sum_z
	# joyCommand.linear.x = Kpx*error_x + Kdx*error_dot_x + Kix*error_sum_x
	# joyCommand.linear.y = Kpx*error_y + Kdx*error_dot_y + Kix*error_sum_y
	# prev_error = error
	# z += 2
	# joyCommand.linear.z = z
	


	print(joyCommand.linear.z)
	joyCommand.linear.z = min(60000,joyCommand.linear.z)
	joyCommand.linear.z = max(33000,joyCommand.linear.z)
	print(joyCommand.linear.z)

	pub.publish(joyCommand)
	rospy.loginfo("sent command")



rospy.init_node("control_crazyflie")
rospy.loginfo("program started")
pub = rospy.Publisher("/crazyflie/cmd_vel", Twist, queue_size=10)
joyCommand = Twist()
rate = rospy.Rate(10)
rospy.Subscriber("/optitrack/rigid_bodies", RigidBodyArray, optitrackCallback)

rigidBodyIdx = 0


rospy.spin()
