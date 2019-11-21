import tf

# get x,y,z,roll,pitch,yaw information of a rigid body
# input: rigit body data of an object from optitrack
# output: [x,y,z,roll,pitch,yaw] of that object in radiance
def GetPositionInfo(cflPose):
	cflPosition = cflPose.position
	cflOrientation = cflPose.orientation

	cflPos_x = cflPosition.x
	cflPos_y = cflPosition.y
	cflPos_z = cflPosition.z
	cflAng_x = cflOrientation.x
	cflAng_y = cflOrientation.y
	cflAng_z = cflOrientation.z
	cflAng_w = cflOrientation.w

	eulerAngles = tf.transformations.euler_from_quaternion([cflAng_x, cflAng_y, cflAng_z, cflAng_w])

	return [cflPos_x,cflPos_y,cflPos_z,eulerAngles[0],eulerAngles[1],eulerAngles[2]]
