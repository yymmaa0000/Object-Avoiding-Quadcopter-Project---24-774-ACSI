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

g = 9.81
r_drone = 0.06
r_ball = 0.25
detect_radius = 2.5
predict_time = 2.0
time_step = 0.5
safety_factor = 2.0

prev_ball_location = [0.0,0.0,0.0]
last_reference = [0.0,0.0,0.0]
last_time = time.time()

def distance(x1,y1,z1,x2,y2,z2):
    temp = (x1-x2)**2+(y1-y2)**2+(z1-z2)**2
    return temp**0.5

def dodge2D(drone_x,drone_y,drone_z,ball_x,ball_y,ball_z,ball_speed_x, ball_speed_y,ball_speed_z):
    position_vector_x= drone_x-ball_x
    position_vector_y= drone_y-ball_y
    
    speed_vector_norm = (ball_speed_x**2+ball_speed_y**2)**0.5
    ball_speed_x /= speed_vector_norm
    ball_speed_y /= speed_vector_norm
    
    projection = position_vector_x*ball_speed_x + position_vector_y*ball_speed_y
    
    perpendicular_x = position_vector_x - ball_speed_x*projection
    perpendicular_y = position_vector_y - ball_speed_y*projection
    
    perpendicular_norm = (perpendicular_x**2+perpendicular_y**2)**0.5
    
    # dx = perpendicular_x / perpendicular_norm * (r_drone+r_ball) * safety_factor
    # dy = perpendicular_y / perpendicular_norm * (r_drone+r_ball) * safety_factor
    dx = perpendicular_x / perpendicular_norm * (r_drone+r_ball) * 1
    dy = perpendicular_y / perpendicular_norm * (r_drone+r_ball) * 1
    
    return (drone_x+dx,drone_y+dy,drone_z)

def dodge3D(drone_x,drone_y,drone_z,ball_x,ball_y,ball_z,ball_speed_x, ball_speed_y,ball_speed_z):
    position_vector_x= drone_x-ball_x
    position_vector_y= drone_y-ball_y
    position_vector_z= drone_z-ball_z
    
    speed_vector_norm = (ball_speed_x**2+ball_speed_y**2+ball_speed_z**2)**0.5
    ball_speed_x /= speed_vector_norm
    ball_speed_y /= speed_vector_norm
    ball_speed_z /= speed_vector_norm
    
    projection = position_vector_x*ball_speed_x + position_vector_y*ball_speed_y +position_vector_z*ball_speed_z 
    
    perpendicular_x = position_vector_x - ball_speed_x*projection
    perpendicular_y = position_vector_y - ball_speed_y*projection
    perpendicular_z = position_vector_z - ball_speed_z*projection
    
    perpendicular_norm = (perpendicular_x**2+perpendicular_y**2+perpendicular_z**2)**0.5
    
    # dx = perpendicular_x / perpendicular_norm * (r_drone+r_ball) * safety_factor
    # dy = perpendicular_y / perpendicular_norm * (r_drone+r_ball) * safety_factor
    # dz = perpendicular_z / perpendicular_norm * (r_drone+r_ball) * safety_factor
    dx = perpendicular_x / perpendicular_norm * (r_drone+r_ball) * 1
    dy = perpendicular_y / perpendicular_norm * (r_drone+r_ball) * 1
    dz = perpendicular_z / perpendicular_norm * (r_drone+r_ball) * 1
    
    return (drone_x+dx,drone_y+dy,drone_z+dz)

# calculate project of vector 2 onto vector 1 (not need to be unit vector)
# return projection and vector perpendicular to vector 2
def project(v1_x,v1_y,v1_z,v2_x,v2_y,v2_z):
    v1_norm = (v1_x**2+v1_y**2+v1_z**2)**0.5
    v1_unit_x = v1_x / v1_norm
    v1_unit_y = v1_y / v1_norm
    v1_unit_z = v1_z / v1_norm
    
    projection = v2_x*v1_unit_x + v2_y*v1_unit_y +v2_z*v1_unit_z 
    
    perpendicular_x = v2_x - v1_unit_x*projection
    perpendicular_y = v2_y - v1_unit_y*projection
    perpendicular_z = v2_z - v1_unit_z*projection

    return projection,perpendicular_x,perpendicular_y,perpendicular_z


# plan a new reference to dodge moving obstacles
# cflPose_drone = rigit body data of the drone from optitrack
# cflPose_obstacle = rigit body data of the obstacle from optitrack
# default_reference = [x,y,z] default reference position when the drone is safe
def plan(cflPose_drone,cflPose_obstacle,default_reference):
    global last_time,last_reference,prev_ball_location

    default_reference = [-0.709,1.755,0.653]

    drone_location = utility_function.GetPositionInfo(cflPose_drone)
    drone_x = drone_location[0]
    drone_y = drone_location[1]
    drone_z = drone_location[2]
    
    ball_location = utility_function.GetPositionInfo(cflPose_obstacle)
    ball_x = ball_location[0]
    ball_y = ball_location[1]
    ball_z = ball_location[2]

    # curr_time = time.time()
    # dt = curr_time-last_time
    # last_time = curr_time

    ball_speed_x = ball_x - prev_ball_location[0]
    ball_speed_y = ball_y - prev_ball_location[1]
    ball_speed_z = ball_z - prev_ball_location[2]

    prev_ball_location[0] = ball_x
    prev_ball_location[1] = ball_y
    prev_ball_location[2] = ball_z

    if (ball_speed_x**2+ball_speed_y**2+ball_speed_z**2)**0.5 < 0.001: 
        last_reference = default_reference
        return default_reference

    if distance(drone_x,drone_y,drone_z,ball_x,ball_y,ball_z) > detect_radius:
        last_reference = default_reference
        return default_reference

    # calculate drone's distance to ball's speed vector
    position_vector_x= drone_x-ball_x
    position_vector_y= drone_y-ball_y
    position_vector_z= drone_z-ball_z

    projection,perpendicular_x,perpendicular_y,perpendicular_z = project(ball_speed_x,
        ball_speed_y,ball_speed_z,position_vector_x,position_vector_y,position_vector_z)
    
    print("projection: ", projection)

    if projection < 0: 
        last_reference = default_reference
        return default_reference

    distance_to_speed_vector = (perpendicular_x**2+perpendicular_y**2+perpendicular_z**2)**0.5
    print("distance_to_speed_vector: ", distance_to_speed_vector)

    if distance_to_speed_vector <= (r_drone+r_ball)*safety_factor:
        position_vector_x= last_reference[0]-ball_x
        position_vector_y= last_reference[1]-ball_y
        position_vector_z= last_reference[2]-ball_z
        _,perpendicular_x,perpendicular_y,perpendicular_z = project(ball_speed_x,ball_speed_y,ball_speed_z,position_vector_x,position_vector_y,position_vector_z)
        last_reference_distance = (perpendicular_x**2+perpendicular_y**2+perpendicular_z**2)**0.5

        if last_reference_distance  <= (r_drone+r_ball)*safety_factor:
            new_reference = dodge3D(drone_x,drone_y,drone_z,ball_x,ball_y,ball_z,ball_speed_x, ball_speed_y,ball_speed_z)
            last_reference = new_reference
            # print("*************************************************new reference :", new_reference)
            return new_reference
        else: return last_reference
    
    last_reference = default_reference
    return default_reference
