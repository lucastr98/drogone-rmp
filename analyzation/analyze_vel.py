import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import math

# FOR USER TO CHANGE
z_distance_to_target = 5
vel_in_x = 1

# calculate values of how camera is mounted on drone
roll_C_B = 0 * np.pi / 180
pitch_C_B = 0 * np.pi / 180
yaw_C_B = 0 * np.pi / 180
t_C_B = np.array([0.0, 0.0, 0.0])

# define camera matrix
f_x = 1140
f_y = 1140
u_0 = 0
v_0 = 0
u_lim = 1024
v_lim = 768

K = np.array([[f_x, 0, u_0, 0],
              [0, f_y, v_0, 0],
              [0, 0, 1, 0]])

# define rotation matrix from camera to body
R_C_B = np.zeros((3, 3))
R_C_B[0][0] = math.cos(pitch_C_B) * math.cos(yaw_C_B)
R_C_B[0][1] = math.cos(pitch_C_B) * math.sin(yaw_C_B)
R_C_B[0][2] = -math.sin(pitch_C_B)
R_C_B[1][0] = math.sin(roll_C_B) * math.sin(pitch_C_B) * math.cos(yaw_C_B) - math.cos(roll_C_B) * math.sin(yaw_C_B)
R_C_B[1][1] = math.sin(roll_C_B) * math.sin(pitch_C_B) * math.sin(yaw_C_B) + math.cos(roll_C_B) * math.cos(yaw_C_B)
R_C_B[1][2] = math.sin(roll_C_B) * math.cos(pitch_C_B)
R_C_B[2][0] = math.cos(roll_C_B) * math.sin(pitch_C_B) * math.cos(yaw_C_B) + math.sin(roll_C_B) * math.sin(yaw_C_B)
R_C_B[2][1] = math.cos(roll_C_B) * math.sin(pitch_C_B) * math.sin(yaw_C_B) - math.sin(roll_C_B) * math.cos(yaw_C_B)
R_C_B[2][2] = math.cos(roll_C_B) * math.cos(pitch_C_B)
trans_C_B = -R_C_B.dot(t_C_B)

# define Transformation matrix
T_C_B = np.identity((4))
T_C_B[0][0] = R_C_B[0][0]
T_C_B[0][1] = R_C_B[0][1]
T_C_B[0][2] = R_C_B[0][2]
T_C_B[0][3] = trans_C_B[0]
T_C_B[1][0] = R_C_B[1][0]
T_C_B[1][1] = R_C_B[1][1]
T_C_B[1][2] = R_C_B[1][2]
T_C_B[1][3] = trans_C_B[1]
T_C_B[2][0] = R_C_B[2][0]
T_C_B[2][1] = R_C_B[2][1]
T_C_B[2][2] = R_C_B[2][2]
T_C_B[2][3] = trans_C_B[2]
T_C_B[3][0] = 0
T_C_B[3][1] = 0
T_C_B[3][2] = 0
T_C_B[3][3] = 1


roll_B_W = 0.0 * np.pi / 180
pitch_B_W = 0.0 * np.pi / 180
yaw_B_W = 0.0 * np.pi / 180
t_B_W = np.array([0.0, 0.0, 0.0])

R_B_W = np.zeros((3, 3))
R_B_W[0][0] = math.cos(pitch_B_W) * math.cos(yaw_B_W)
R_B_W[0][1] = math.cos(pitch_B_W) * math.sin(yaw_B_W)
R_B_W[0][2] = -math.sin(pitch_B_W)
R_B_W[1][0] = math.sin(roll_B_W) * math.sin(pitch_B_W) * math.cos(yaw_B_W) - math.cos(roll_B_W) * math.sin(yaw_B_W)
R_B_W[1][1] = math.sin(roll_B_W) * math.sin(pitch_B_W) * math.sin(yaw_B_W) + math.cos(roll_B_W) * math.cos(yaw_B_W)
R_B_W[1][2] = math.sin(roll_B_W) * math.cos(pitch_B_W)
R_B_W[2][0] = math.cos(roll_B_W) * math.sin(pitch_B_W) * math.cos(yaw_B_W) + math.sin(roll_B_W) * math.sin(yaw_B_W)
R_B_W[2][1] = math.cos(roll_B_W) * math.sin(pitch_B_W) * math.sin(yaw_B_W) - math.sin(roll_B_W) * math.cos(yaw_B_W)
R_B_W[2][2] = math.cos(roll_B_W) * math.cos(pitch_B_W)
trans_B_W = -R_B_W.dot(t_B_W)

T_B_W = np.identity((4))
T_B_W[0][0] = R_B_W[0][0]
T_B_W[0][1] = R_B_W[0][1]
T_B_W[0][2] = R_B_W[0][2]
T_B_W[0][3] = trans_B_W[0]
T_B_W[1][0] = R_B_W[1][0]
T_B_W[1][1] = R_B_W[1][1]
T_B_W[1][2] = R_B_W[1][2]
T_B_W[1][3] = trans_B_W[1]
T_B_W[2][0] = R_B_W[2][0]
T_B_W[2][1] = R_B_W[2][1]
T_B_W[2][2] = R_B_W[2][2]
T_B_W[2][3] = trans_B_W[2]
T_B_W[3][0] = 0
T_B_W[3][1] = 0
T_B_W[3][2] = 0
T_B_W[3][3] = 1

target_W = np.array([0, 0, z_distance_to_target, 1])
target_C = T_C_B.dot(T_B_W.dot(target_W))
u_v = K.dot(target_C)
normalization = u_v[2]

K_vel = np.array([[f_x, 0, u_0],
                  [0, f_y, v_0],
                  [0, 0, 1]])

target_vel_W = np.array([vel_in_x, 0, 0])
target_vel_C = R_C_B.dot(R_B_W.dot(target_vel_W))
u_v_dot_unnorm = K_vel.dot(target_vel_C)
u_v_dot = u_v_dot_unnorm / normalization
print(u_v_dot)
