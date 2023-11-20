import numpy as np
from time import sleep

np.set_printoptions(precision=3, suppress=True) #Set precision

#State Space Model Initialization Variables
A_t_minus_1 = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) #The A matrix expresses how the state of the system [x position,y position,yaw angle γ] changes from t-1 to t when no control command is executed (I is the ideal state, need to account for gravity on a downhill)

X_t_minus_1 = np.array([[0], [0], [0]]) #The X matrix (x, y, yaw) expresses the state of the rover at the given time-step (Rover starts from rest and is at the origin)

u_t_minus_1 = np.array([[0], [0]]) #The u matrix represents the control input given to the rover at the given time-step, i.e. the linear and angular velocity v and omega respectively

sigma_p = np.array([[0], [0], [0]]) #Process Noise (Tweak by trial and error)

yaw, dt = 0, 1 #Initial yaw angle and time-step

def getB(yaw, dt):
    
    B = np.array([[np.cos(yaw)*dt, 0], [np.sin(yaw)*dt, 0], [0, dt]]) #Expresses how the state of the system [x,y,yaw] changes from t-1 to t due to the control commands (i.e. control input).

    return B

def X_t(): #State Space Model
    B_t_minus_1 = getB(yaw, dt)

    X = (A_t_minus_1 @ X_t_minus_1) + (B_t_minus_1 @ u_t_minus_1) + sigma_p #State of the rover at time-step t

    return X

while True:
    print(X_t())
    sleep(0.1)

