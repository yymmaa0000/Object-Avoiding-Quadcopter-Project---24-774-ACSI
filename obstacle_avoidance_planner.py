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
safety_factor = 1.0

prev_ball_location = [0.0,0.0,0.0]
last_reference = [0.0,0.0,0.0]
last_time = time.time()

def distance(x1,y1,z1,x2,y2,z2):
    temp = (x1-x2)**2+(y1-y2)**2+(z1-z2)**2
    return temp**0.5

def dodge2D(drone_x,drone_y,drone_z,ball_x,ball_y,ball_z,ball_speed_x, ball_speed_y,ball_speed_z):
    position_vector_x= drone_x-ball_x
    position_vector_y= drone_y-ball_y
    
    speend_vector_norm = (ball_speed_x**2+ball_speed_y**2)**0.5
    ball_speed_x /= speend_vector_norm
    ball_speed_y /= speend_vector_norm
    
    projection = position_vector_x*ball_speed_x + position_vector_y*ball_speed_y
    
    perpendicular_x = position_vector_x - ball_speed_x*projection
    perpendicular_y = position_vector_y - ball_speed_y*projection
    
    perpendicular_norm = (perpendicular_x**2+perpendicular_y**2)**0.5
    
    dx = perpendicular_x / perpendicular_norm * (r_drone+r_ball) * safety_factor
    dy = perpendicular_y / perpendicular_norm * (r_drone+r_ball) * safety_factor
    
    return (drone_x+dx,drone_y+dy,drone_z)

def dodge3D(drone_x,drone_y,drone_z,ball_x,ball_y,ball_z,ball_speed_x, ball_speed_y,ball_speed_z):
    position_vector_x= drone_x-ball_x
    position_vector_y= drone_y-ball_y
    position_vector_z= drone_z-ball_z
    
    speend_vector_norm = (ball_speed_x**2+ball_speed_y**2+ball_speed_z**2)**0.5
    ball_speed_x /= speend_vector_norm
    ball_speed_y /= speend_vector_norm
    ball_speed_z /= speend_vector_norm
    
    projection = position_vector_x*ball_speed_x + position_vector_y*ball_speed_y +position_vector_z*ball_speed_z 
    
    perpendicular_x = position_vector_x - ball_speed_x*projection
    perpendicular_y = position_vector_y - ball_speed_y*projection
    perpendicular_z = position_vector_z - ball_speed_z*projection
    
    perpendicular_norm = (perpendicular_x**2+perpendicular_y**2+perpendicular_z**2)**0.5
    
    dx = perpendicular_x / perpendicular_norm * (r_drone+r_ball) * safety_factor
    dy = perpendicular_y / perpendicular_norm * (r_drone+r_ball) * safety_factor
    dz = perpendicular_z / perpendicular_norm * (r_drone+r_ball) * safety_factor
    
    return (drone_x+dx,drone_y+dy,drone_z+dz)

# plan a new reference to dodge moving obstacles
# cflPose_drone = rigit body data of the drone from optitrack
# cflPose_obstacle = rigit body data of the obstacle from optitrack
# default_reference = [x,y,z] default reference position when the drone is safe
def plan(cflPose_drone,cflPose_obstacle,default_reference):
    global last_time,last_reference,prev_ball_location

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
    dt = 0.005

    ball_speed_x = (ball_x - prev_ball_location[0])/dt
    ball_speed_y = (ball_y - prev_ball_location[1])/dt
    ball_speed_z = (ball_z - prev_ball_location[2])/dt

    prev_ball_location[0] = ball_x
    prev_ball_location[1] = ball_y
    prev_ball_location[2] = ball_z

    if distance(drone_x,drone_y,drone_z,ball_x,ball_y,ball_z) > detect_radius:
        last_reference = default_reference
        return default_reference
    
    t = 0
    ball_x_future = ball_x
    ball_y_future = ball_y
    ball_z_future = ball_z
    while t < predict_time:
        ball_speed_z -= time_step*g
        ball_x_future += ball_speed_x*time_step
        ball_y_future += ball_speed_y*time_step
        ball_z_future += ball_speed_z*time_step
        
        d = distance(drone_x,drone_y,drone_z,ball_x_future,ball_y_future,ball_z_future)
        
        if d > detect_radius: break
        if d <= (r_drone+r_ball): 
            if distance(last_reference[0],last_reference[1],last_reference[2],ball_x_future,ball_y_future,ball_z_future) <= (r_drone+r_ball): 
                new_reference = dodge2D(drone_x,drone_y,drone_z,ball_x_future,ball_y_future,ball_z_future,ball_speed_x, ball_speed_y,ball_speed_z)
                last_reference = new_reference
                print("new reference :", new_reference)
                return new_reference
            else: return last_reference
        t += time_step
    
    last_reference = default_reference
    return default_reference


# if __name__ == "__main__":
#     drone_x = 0
#     drone_y = 0
#     drone_z = 2
    
#     ball_x = 2.05
#     ball_y = 1.95
#     ball_z = 1.5
    
#     ball_x_prev = 3
#     ball_y_prev = 3
#     ball_z_prev = 1
    
#     dt = 0.1
    
#     fig = plt.figure()
#     ax = plt.axes(projection="3d")
    
#     ax.scatter3D(drone_x, drone_y, drone_z,  c='b',label = "Drone original potition")
    
#     x = [ball_x]
#     y = [ball_y]
#     z = [ball_z]
#     x_hit = []
#     y_hit = []
#     z_hit = []
#     ball_speed_x = (ball_x - ball_x_prev)/dt
#     ball_speed_y = (ball_y - ball_y_prev)/dt
#     ball_speed_z = (ball_z - ball_z_prev)/dt
#     t = 0
#     ball_x_future = ball_x
#     ball_y_future = ball_y
#     ball_z_future = ball_z
    
#     dodge = True
#     while t < predict_time:
#         ball_speed_z -= time_step*g
#         ball_x_future += ball_speed_x*time_step
#         ball_y_future += ball_speed_y*time_step
#         ball_z_future += ball_speed_z*time_step
        
#         d = distance(drone_x,drone_y,drone_z,ball_x_future,ball_y_future,ball_z_future)
#         if d > detect_radius: break
#         if d <= (r_drone+r_ball): 
#             x_hit.append(ball_x_future)
#             y_hit.append(ball_y_future)
#             z_hit.append(ball_z_future)
#             if dodge:
#                 a,b,c = dodge3D(drone_x,drone_y,drone_z,ball_x_future,ball_y_future,ball_z_future,ball_speed_x, ball_speed_y,ball_speed_z)
#                 ax.scatter3D(a,b,c,  c='y',label = "Drone position after dodging (3D)")
#                 print("dodge3D: ",distance(drone_x,drone_y,drone_z,a,b,c))
                
#                 a,b,c = dodge2D(drone_x,drone_y,drone_z,ball_x_future,ball_y_future,ball_z_future,ball_speed_x, ball_speed_y,ball_speed_z)
#                 ax.scatter3D(a,b,c,  c='g',label = "Drone position after dodging (2D)")
#                 print("dodge2D: ",distance(drone_x,drone_y,drone_z,a,b,c))
                
#                 dodge = False
        
#         x.append(ball_x_future)
#         y.append(ball_y_future)
#         z.append(ball_z_future)
#         t += time_step
    
#     ax.plot3D(x, y, z, 'black',label = "Predicted ball trajectory")
#     ax.plot3D(x_hit, y_hit, z_hit, 'red',label = "Collision region")
#     ax.set_xlabel('x')
#     ax.set_ylabel('y')
#     ax.set_zlabel('z');
#     ax.set_xlim3d(-5, 5)
#     ax.set_ylim3d(-5,5)
#     ax.set_zlim3d(0,3)
#     plt.legend(loc=2)
#     plt.show()