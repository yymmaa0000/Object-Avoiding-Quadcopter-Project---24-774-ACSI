"""
24-774 Obstacle Avoidance Drone Project
obstacle_avoidance_planner for dodging moving obstacles
Created on Sun Nov 10 15:13:30 2019
author: XingYu Wang
"""
import utility_function
# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits import mplot3d

activation_dis = 0.1
max_distance = 3
min_distance = 0.5
arm_length = 0.8

# plan a new reference to following hand movement
# cflPose_drone = rigit body data of the arm from optitrack
# cflPose_obstacle = rigit body data of the shoulder from optitrack
# default_reference = [x,y,z] default reference position when the drone is safe
def plan(cflPose_arm,cflPose_shoulder,default_reference):
    arm_location = utility_function.GetPositionInfo(cflPose_arm)
    shoulder_location = utility_function.GetPositionInfo(cflPose_shoulder)

    if (shoulder_location[2] - arm_location[2] > activation_dis): return default_reference

    dx = arm_location[0] - shoulder_location[0]
    dy = arm_location[1] - shoulder_location[1]
    dz = arm_location[2] - shoulder_location[2]

    dis = (dx*dx+dy*dy+dz*dz)**(0.5)
    new_length = dis/arm_length*(max_distance-min_distance)+min_distance

    new_x = shoulder_location[0] + dx/dis*new_length
    new_y = shoulder_location[1] + dy/dis*new_length
    new_z = shoulder_location[2] + dz/dis*new_length

    return [new_x,new_y,new_z]

