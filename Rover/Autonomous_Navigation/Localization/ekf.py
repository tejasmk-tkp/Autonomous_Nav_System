import numpy as np
import matplotlib as plt

def ekf(yaw, dt, A_t_minus_1, B_t_minus_1, X_t_minus_1, U_t_minus_1, process_noise, Q_t, P_t_minus_1, H_t, sensor_error, R_t, sensor_reading):
''' A - Component of State Estimate
    B - Component of control input
    X - State estimate of bot
    U - Control Input
    Q - Noise covariance
    dt - time-step
    P - Pridicted State Covariance
    Y - Future State Estimate
    H - Measurement Matrix
    S - Residual Covariance
    R - Sensor measurement noise covariance matrix
    K - Kalman Gain'''
    
    X_t = (A_t_minus_1 @ X_t_minus_1) + (B_t_minus_1 @ U_t_minus_1) + process_noise
    P_t = (A_t_minus_1 @ P_t_minus_1 @ np.transpose(A_t_minus_1)) + Q_t
    Y_t = (H_t @ X_t) + sensor_error
    Y_t_updated = sensor_reading - Y_t
    S_t = (H_t @ P_t @ np.transpose(H_t)) + R_t
    K_t = P_t @ np.transpose(H_t) @ np.linalg.inv(S_t)
    X_t_updated = X_t + (K_t @ Y_t_updated)
    P_t_updated = (np.eye(3) - (K_t @ H_t)) @ P_t

    return X_t_updated

def visualize_point(X_t_updated):
    x, y, yaw = X_t_updated[0]

    arrow_length = 1.5
    arrow_x = x + arrow_length * np.cos(yaw)
    arrow_y = y + arrow_length * np.sin(yaw)

    plt.plot(x, y, 'ro')

    plt.arrow(x, y, arrow_x - x, arrow_y - y, head_width = 0.1, head_length = 0.1, fc = 'blue', ec = 'blue')

    plt.show()
