# -*- coding: utf-8 -*-
"""
@author: XingYu Wang
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d


file = open("data1.txt","r") 
raw_data = file.readlines()
file.close()
dt = 0.0162
raw_data = raw_data[180:]
x = []
y = []
z = []

for a in raw_data:
    b = a.split()
    x.append(float(b[0]))
    y.append(float(b[1]))
    z.append(float(b[2]))
    
# use linear regression to fit the first few points of the actual data
n = 4
x_sample = x[:n]
y_sample = y[:n]
z_sample = z[:n]  

Y = np.array(z_sample).reshape(-1, 1)
X1 = np.array(x_sample).reshape(-1, 1)
X2 = np.array(y_sample).reshape(-1, 1)
X0 = np.ones((n,1))
X = np.hstack((np.hstack((X0,X1)),X2))
B = np.linalg.inv((np.transpose(X).dot(X))).dot(np.transpose(X)).dot(Y)

ball_x_prev = x_sample[0]
ball_y_prev = y_sample[1]
ball_z_prev = float(B[0] + ball_x_prev*B[1] + ball_y_prev*B[2])

ball_x = x_sample[-1]
ball_y = y_sample[-1]
ball_z = float(B[0] + ball_x*B[1] + ball_y*B[2])

# make prediction
predict_time = 0.8
time_step = 0.01

x_predict = [ball_x]
y_predict = [ball_y]
z_predict = [ball_z]
ball_speed_x = (ball_x - ball_x_prev)/dt
ball_speed_y = (ball_y - ball_y_prev)/dt
ball_speed_z = (ball_z - ball_z_prev)/dt

ball_x_future = ball_x
ball_y_future = ball_y
ball_z_future = ball_z

t = 0
g = 9.81
while t < predict_time:
    ball_speed_z -= time_step*g
    ball_x_future += ball_speed_x*time_step
    ball_y_future += ball_speed_y*time_step
    ball_z_future += ball_speed_z*time_step
    x_predict.append(ball_x_future)
    y_predict.append(ball_y_future)
    z_predict.append(ball_z_future)
    t += time_step
    
    
fig = plt.figure()
ax = plt.axes(projection="3d")
ax.scatter3D(x_sample, y_sample, z_sample,  c='b',label = "Points used for making prediction")
#ax.scatter3D(ball_x, ball_y, ball_z,  c='r',label = "Drone original potition")
ax.plot3D(x, y, z, 'black',label = "Actual ball trajectory")
ax.plot3D(x_predict, y_predict, z_predict, 'red',label = "Predicted ball trajectory")
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_zlabel('z (m)');
#ax.set_xlim3d(-4, 4)
#ax.set_ylim3d(-4,4)
#ax.set_zlim3d(0,3)
#plt.legend(loc=2)
plt.show()





#
#if __name__ == "__main__":
#    drone_x = 0
#    drone_y = 0
#    drone_z = 2
#    
#    ball_x = 2.05
#    ball_y = 1.95
#    ball_z = 1.5
#    
#    ball_x_prev = 3
#    ball_y_prev = 3
#    ball_z_prev = 1
#    
#    dt = 0.1
#    
#    fig = plt.figure()
#    ax = plt.axes(projection="3d")
#    
#    ax.scatter3D(drone_x, drone_y, drone_z,  c='b',label = "Drone original potition")
#    
#    x = [ball_x]
#    y = [ball_y]
#    z = [ball_z]
#    x_hit = []
#    y_hit = []
#    z_hit = []
#    ball_speed_x = (ball_x - ball_x_prev)/dt
#    ball_speed_y = (ball_y - ball_y_prev)/dt
#    ball_speed_z = (ball_z - ball_z_prev)/dt
#    t = 0
#    ball_x_future = ball_x
#    ball_y_future = ball_y
#    ball_z_future = ball_z
#    
#    dodge = True
#    while t < predict_time:
#        ball_speed_z -= time_step*g
#        ball_x_future += ball_speed_x*time_step
#        ball_y_future += ball_speed_y*time_step
#        ball_z_future += ball_speed_z*time_step
#        
#        d = distance(drone_x,drone_y,drone_z,ball_x_future,ball_y_future,ball_z_future)
#        if d > detect_radius: break
#        if d <= (r_drone+r_ball): 
#            x_hit.append(ball_x_future)
#            y_hit.append(ball_y_future)
#            z_hit.append(ball_z_future)
#            if dodge:
#                a,b,c = dodge3D(drone_x,drone_y,drone_z,ball_x_future,ball_y_future,ball_z_future,ball_speed_x, ball_speed_y,ball_speed_z)
#                ax.scatter3D(a,b,c,  c='y',label = "Drone position after dodging (3D)")
#                print("dodge3D: ",distance(drone_x,drone_y,drone_z,a,b,c))
#                
#                a,b,c = dodge2D(drone_x,drone_y,drone_z,ball_x_future,ball_y_future,ball_z_future,ball_speed_x, ball_speed_y,ball_speed_z)
#                ax.scatter3D(a,b,c,  c='g',label = "Drone position after dodging (2D)")
#                print("dodge2D: ",distance(drone_x,drone_y,drone_z,a,b,c))
#                
#                dodge = False
#        
#        x.append(ball_x_future)
#        y.append(ball_y_future)
#        z.append(ball_z_future)
#        t += time_step
#    
#    ax.plot3D(x, y, z, 'black',label = "Predicted ball trajectory")
#    ax.plot3D(x_hit, y_hit, z_hit, 'red',label = "Collision region")
#    ax.set_xlabel('x')
#    ax.set_ylabel('y')
#    ax.set_zlabel('z');
#    ax.set_xlim3d(-5, 5)
#    ax.set_ylim3d(-5,5)
#    ax.set_zlim3d(0,3)
##    plt.legend(loc=2)
#    plt.show()