import math
import utility_function

# linear x,y = [-30,30] 
# linear z = [0,60000]
# angular x,y = 0
# angular z = [-200,200]

# warning, please make sure that the yaw angle measured from optitrack system
# matches the yaw angle represented by the actual space!

PI = 3.1415927 

prev_error = [0.0,0.0,0.0]
error_sum = [0.0,0.0,0.0]
Hover_Speed = 33000.0

Kp = [10.0,10.0,30000.0]
Kd = [50.0,50.0,30000.0]
Ki = [2.0,2.0,0.0]

def calculateXYerror(targetX,targetY,DroneX,DroneY,DroneYaw):
	dx = targetX - DroneX
	dy = targetY - DroneY
	y_error = dx*math.sin(DroneYaw) - dy * math.cos(DroneYaw)
	x_error = dx*math.cos(DroneYaw) + dy * math.sin(DroneYaw)
	return x_error, y_error

# calculate controller output using PID
# desired_location = [x,y,z] location in space
# cflPose = rigit body data of the drone from optitrack
# dt = time step
# output: all 6 joystick commpand
def ControlOutput(desired_location,cflPose,dt):
	x,y,z,roll,pitch,yaw = utility_function.GetPositionInfo(cflPose)

	print("=======================================================================================")
	print("Position X = ", x)
	print("Position Y = ", y)
	print("Position Z = ", z)
	print("Roll = ", roll/PI*180)
	print("Pitch = ", pitch/PI*180)
	print("Yaw = ", yaw/PI*180)

	joyCommand_linear_x = 0.0
	joyCommand_linear_y = 0.0
	joyCommand_linear_z = 50000.0
	joyCommand_angular_x = 0.0
	joyCommand_angular_y = 0.0
	joyCommand_angular_z = 0.0
	
	error = [0.0]*3
	error_dot = [0.0]*3
	error[0], error[1] = calculateXYerror(desired_location[0],desired_location[1],x,y,yaw)
	error[2] = desired_location[2] - z
	
	for i in range(3):
		error_dot[i] = (error[i] - prev_error[i])/dt
		error_sum[i] += error[i] * dt
	
	joyCommand_linear_x = Kp[0]*error[0] + Kd[0]*error_dot[0] + Ki[0]*error_sum[0]
	joyCommand_linear_y = Kp[1]*error[1] + Kd[1]*error_dot[1] + Ki[1]*error_sum[1]
	joyCommand_linear_z = Kp[2]*error[2] + Kd[2]*error_dot[2] + Ki[2]*error_sum[2]
	joyCommand_linear_z += Hover_Speed
	
	for i in range(3):
		prev_error[i] = error[i]
	
	joyCommand_linear_x = min(30,joyCommand_linear_x)
	joyCommand_linear_x = max(-30,joyCommand_linear_x)
	joyCommand_linear_y = min(30,joyCommand_linear_y)
	joyCommand_linear_y = max(-30,joyCommand_linear_y)
	joyCommand_linear_z = min(60000,joyCommand_linear_z)
	joyCommand_linear_z = max(Hover_Speed,joyCommand_linear_z)

	print("Error X: ",error[0]," Error Y: ",error[1]," Error Z: ",error[2])
	print("Joy command X: ",joyCommand_linear_x," Joy command Y: ",joyCommand_linear_y," Joy command Z: ",joyCommand_linear_z)

	return [joyCommand_linear_x,joyCommand_linear_y,joyCommand_linear_z,joyCommand_angular_x,joyCommand_angular_y,joyCommand_angular_z]