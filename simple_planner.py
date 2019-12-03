"""
24-774 Obstacle Avoidance Drone Project
obstacle_avoidance_planner for dodging moving obstacles
Created on Sun Nov 10 15:13:30 2019
author: XingYu Wang
"""
import utility_function
import time
# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits import mplot3d

detect_distance = 2.0
d_diff_threshold = -0.005
movement = -1.2 # move to potitive x direction
old_d = 0

def distance(x1,y1,z1,x2,y2,z2):
    temp = (x1-x2)**2+(y1-y2)**2+(z1-z2)**2
    return temp**0.5

# plan a new reference to dodge moving obstacles
# cflPose_drone = rigit body data of the drone from optitrack
# cflPose_obstacle = rigit body data of the obstacle from optitrack
# default_reference = [x,y,z] default reference position when the drone is safe
def plan(cflPose_drone,cflPose_obstacle,default_reference):
    global old_d

    drone_location = utility_function.GetPositionInfo(cflPose_drone)
    drone_x = drone_location[0]
    drone_y = drone_location[1]
    drone_z = drone_location[2]
    
    ball_location = utility_function.GetPositionInfo(cflPose_obstacle)
    ball_x = ball_location[0]
    ball_y = ball_location[1]
    ball_z = ball_location[2]

    d = distance(drone_x,drone_y,drone_z,ball_x,ball_y,ball_z)
    d_diff = d - old_d
    old_d = d

    if d < detect_distance or d_diff < d_diff_threshold: 
        return [default_reference[0],default_reference[1]+movement,default_reference[2]]
    else: return default_reference