import numpy as np
import matplotlib.pyplot as plt
import rosbag
import yaml
import math

# load bags
bags = []
with open(r'evaluation.yaml') as file:
    list = yaml.safe_load(file)
    rosbags = list['rosbags']
    for bag in rosbags:
        bags.append(rosbag.Bag(bag + ".bag"))
    time_tot = list['time_tot']
    roll_pitch_zero = list['roll_pitch_zero']

# store uav poses and target positions
uav_poses = []
target_positions = []
for bag in bags:
    for topic, msg, t in bag.read_messages(topics='/firefly/command/trajectory'):
        starting_time = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
        break

    counter = 1
    temp_uav_poses = []
    for topic, msg, t in bag.read_messages(topics='/firefly/odometry_sensor1/odometry'):
        current_time = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
        if(current_time < starting_time):
            continue
        elif(counter > time_tot * 100):
            break
        temp_uav_poses.append(msg.pose.pose)
        counter += 1
    uav_poses.append(temp_uav_poses)

    counter = 1
    temp_target_positions = []
    for topic, msg, t in bag.read_messages(topics='/victim_drone/odometry'):
        current_time = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
        if(current_time < starting_time):
            continue
        elif(counter > time_tot * 100):
            break
        temp_target_positions.append(msg.pose.pose.position)
        counter += 1
    target_positions.append(temp_target_positions)

# define camera constants
f_x = 1140
f_y = 1140
u_0 = 0
v_0 = 0
roll_C_B = 0 * np.pi / 180
pitch_C_B = 0 * np.pi / 180
yaw_C_B = 0 * np.pi / 180
t_C_B = np.array([0.0, 0.0, 0.0])
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

u = []
v = []
px_error = []
xy_error = []
for i in range(0, len(uav_poses)):
    T_B_W = []
    uav_xy = []
    for uav_pose in uav_poses[i]:
        uav_xy.append(np.array([uav_pose.position.x, uav_pose.position.y]))
        if(roll_pitch_zero):
            roll_B_W = 0.0
            pitch_B_W = 0.0
        else:
            roll_B_W = math.atan2(2.0 * (uav_pose.orientation.w * uav_pose.orientation.x + uav_pose.orientation.y * uav_pose.orientation.z),
                                  1.0 - 2.0 * (uav_pose.orientation.x ** 2 - uav_pose.orientation.y ** 2))
            pitch_B_W = 2.0 * (uav_pose.orientation.w * uav_pose.orientation.y - uav_pose.orientation.z * uav_pose.orientation.x)
        yaw_B_W = math.atan2(2.0 * (uav_pose.orientation.w * uav_pose.orientation.z + uav_pose.orientation.x * uav_pose.orientation.y),
                             1.0 - 2.0 * (uav_pose.orientation.y ** 2 - uav_pose.orientation.z ** 2))
        t_B_W = np.array([uav_pose.position.x, uav_pose.position.y, uav_pose.position.z])
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
        temp_T_B_W = np.identity((4))
        temp_T_B_W[0][0] = R_B_W[0][0]
        temp_T_B_W[0][1] = R_B_W[0][1]
        temp_T_B_W[0][2] = R_B_W[0][2]
        temp_T_B_W[0][3] = trans_B_W[0]
        temp_T_B_W[1][0] = R_B_W[1][0]
        temp_T_B_W[1][1] = R_B_W[1][1]
        temp_T_B_W[1][2] = R_B_W[1][2]
        temp_T_B_W[1][3] = trans_B_W[1]
        temp_T_B_W[2][0] = R_B_W[2][0]
        temp_T_B_W[2][1] = R_B_W[2][1]
        temp_T_B_W[2][2] = R_B_W[2][2]
        temp_T_B_W[2][3] = trans_B_W[2]
        temp_T_B_W[3][0] = 0
        temp_T_B_W[3][1] = 0
        temp_T_B_W[3][2] = 0
        temp_T_B_W[3][3] = 1
        T_B_W.append(temp_T_B_W)
    target_W = []
    target_xy = []
    for target_position in target_positions[i]:
        target_xy.append(np.array([target_position.x, target_position.y]))
        target_W.append(np.array([target_position.x, target_position.y, target_position.z, 1]))
    temp_u = []
    temp_v = []
    temp_px_error = []
    temp_xy_error = []
    for j in range(0, len(T_B_W)):
        target_C = T_C_B.dot(T_B_W[j].dot(target_W[j]))
        u_v = K.dot(target_C)
        temp_u.append(u_v[0] / u_v[2])
        temp_v.append(u_v[1] / u_v[2])
        temp_px_error.append(math.sqrt(u_v[0]**2 + u_v[1]**2) / u_v[2])
        temp_xy_error.append(math.sqrt((target_xy[j] - uav_xy[j])[0] ** 2 + (target_xy[j] - uav_xy[j])[1] ** 2))

    u.append(temp_u)
    v.append(temp_v)
    px_error.append(temp_px_error)
    xy_error.append(temp_xy_error)

px_mean = []
px_stddev = []
px_mean_plus_stddev = []
px_mean_minus_stddev = []
xy_mean = []
xy_stddev = []
xy_mean_plus_stddev = []
xy_mean_minus_stddev = []
for i in range(0, len(px_error[0])):
    px_sum = 0
    xy_sum = 0
    for j in range(0, len(px_error)):
        px_sum += px_error[j][i]
        xy_sum += xy_error[j][i]
    px_mean.append(px_sum / len(px_error))
    xy_mean.append(xy_sum / len(xy_error))
    px_stddev_sum = 0
    xy_stddev_sum = 0
    for j in range(0, len(px_error)):
        px_stddev_sum += (px_error[j][i] - px_mean[len(px_mean) - 1]) ** 2
        xy_stddev_sum += (xy_error[j][i] - xy_mean[len(xy_mean) - 1]) ** 2
    px_stddev.append(math.sqrt(px_stddev_sum / len(px_error)))
    xy_stddev.append(math.sqrt(xy_stddev_sum / len(xy_error)))
    px_mean_plus_stddev.append(px_mean[len(px_mean) - 1] + px_stddev[len(px_stddev) - 1])
    xy_mean_plus_stddev.append(xy_mean[len(xy_mean) - 1] + xy_stddev[len(xy_stddev) - 1])
    px_mean_minus_stddev.append(px_mean[len(px_mean) - 1] - px_stddev[len(px_stddev) - 1])
    xy_mean_minus_stddev.append(xy_mean[len(xy_mean) - 1] - xy_stddev[len(xy_stddev) - 1])

fig, axes = plt.subplots(2, 1) #, sharex=True)

time = np.linspace(0, time_tot - 0.01, time_tot * 100)
axes[0].plot(time, px_mean)
axes[0].fill_between(time, px_mean_minus_stddev, px_mean_plus_stddev, alpha=0.5)
axes[0].grid()
axes[1].plot(time, xy_mean)
axes[1].fill_between(time, xy_mean_minus_stddev, xy_mean_plus_stddev, alpha=0.5)
axes[1].grid()
plt.show()
