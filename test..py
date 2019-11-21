from planner import *

dt = 0.1

drone_x = 0
drone_y = 0
drone_z = 2
    
ball_x = 2.05
ball_y = 1.95
ball_z = 1.5
    
ball_x_prev = 3
ball_y_prev = 3
ball_z_prev = 1

result = plan(drone_x,drone_y,drone_z,ball_x,ball_y,ball_z,ball_x_prev,ball_y_prev,ball_z_prev,dt)

print(result)