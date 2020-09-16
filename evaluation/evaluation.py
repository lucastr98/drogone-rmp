import numpy as np
import matplotlib.pyplot as plt
from matplotlib.pyplot import cm
import rosbag
import yaml
import math

# load bags
bags_complete = []
with open(r'evaluation.yaml') as file:
    list = yaml.safe_load(file)
    evaluation = list['evaluation']
    target_velocity = list['target_velocity']
    flight_path = list['flight_path']
    init_px_err = list['init_px_err']
    rosbags = list['rosbags']
    time_tot = list['time_tot']
    roll_pitch_zero = list['roll_pitch_zero']

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

if(evaluation == 'follow'):
    for px_err_val in init_px_err:
        bags = []
        for bag in rosbags:
            bags.append(rosbag.Bag("/home/severin/luca_ws/rosbags/" + evaluation + "_evaluation/vel_" + target_velocity + "/" + flight_path + "/px_err_" + str(px_err_val) + "/" + bag + ".bag"))
            bags_complete.append(bags)

    # store uav poses and target positions
    uav_poses_complete = []
    target_positions_complete = []
    for bags in bags_complete:
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
        uav_poses_complete.append(uav_poses)
        target_positions_complete.append(target_positions)


    u_complete = []
    v_complete = []
    px_error_complete = []
    xy_error_complete = []
    for a in range(0, len(uav_poses_complete)):
        u = []
        v = []
        px_error = []
        xy_error = []
        for i in range(0, len(uav_poses_complete[a])):
            T_B_W = []
            uav_xy = []
            for uav_pose in uav_poses_complete[a][i]:
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
            for target_position in target_positions_complete[a][i]:
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
        u_complete.append(u)
        v_complete.append(v)
        px_error_complete.append(px_error)
        xy_error_complete.append(xy_error)

    px_mean_complete = []
    px_stddev_complete = []
    px_mean_plus_stddev_complete = []
    px_mean_minus_stddev_complete = []
    xy_mean_complete = []
    xy_stddev_complete = []
    xy_mean_plus_stddev_complete = []
    xy_mean_minus_stddev_complete = []
    for a in range(0, len(px_error_complete)):
        px_mean = []
        px_stddev = []
        px_mean_plus_stddev = []
        px_mean_minus_stddev = []
        xy_mean = []
        xy_stddev = []
        xy_mean_plus_stddev = []
        xy_mean_minus_stddev = []
        for i in range(0, len(px_error_complete[a][0])):
            px_sum = 0
            xy_sum = 0
            for j in range(0, len(px_error_complete[a])):
                px_sum += px_error_complete[a][j][i]
                xy_sum += xy_error_complete[a][j][i]
            px_mean.append(px_sum / len(px_error_complete[a]))
            xy_mean.append(xy_sum / len(xy_error_complete[a]))
            px_stddev_sum = 0
            xy_stddev_sum = 0
            for j in range(0, len(px_error_complete[a])):
                px_stddev_sum += (px_error_complete[a][j][i] - px_mean[len(px_mean) - 1]) ** 2
                xy_stddev_sum += (xy_error_complete[a][j][i] - xy_mean[len(xy_mean) - 1]) ** 2
            px_stddev.append(math.sqrt(px_stddev_sum / len(px_error_complete[a])))
            xy_stddev.append(math.sqrt(xy_stddev_sum / len(xy_error_complete[a])))
            px_mean_plus_stddev.append(px_mean[len(px_mean) - 1] + px_stddev[len(px_stddev) - 1])
            xy_mean_plus_stddev.append(xy_mean[len(xy_mean) - 1] + xy_stddev[len(xy_stddev) - 1])
            px_mean_minus_stddev.append(px_mean[len(px_mean) - 1] - px_stddev[len(px_stddev) - 1])
            xy_mean_minus_stddev.append(xy_mean[len(xy_mean) - 1] - xy_stddev[len(xy_stddev) - 1])
        px_mean_complete.append(px_mean)
        px_stddev_complete.append(px_stddev)
        px_mean_plus_stddev_complete.append(px_mean_plus_stddev)
        px_mean_minus_stddev_complete.append(px_mean_minus_stddev)
        xy_mean_complete.append(xy_mean)
        xy_stddev_complete.append(xy_stddev)
        xy_mean_plus_stddev_complete.append(xy_mean_plus_stddev)
        xy_mean_minus_stddev_complete.append(xy_mean_minus_stddev)

    n = len(px_mean_complete)

    fig, axes = plt.subplots(2, 1) #, sharex=True)
    time = np.linspace(0, time_tot - 0.01, time_tot * 100)
    color = iter(cm.rainbow(np.linspace(0, 1, n)))

    for i in range(n):
        c = next(color)
        axes[0].plot(time, px_mean_complete[i], c=c)
        axes[0].fill_between(time, px_mean_minus_stddev_complete[i], px_mean_plus_stddev_complete[i], alpha=0.5, color=c)
        axes[1].plot(time, xy_mean_complete[i], c=c)
        axes[1].fill_between(time, xy_mean_minus_stddev_complete[i], xy_mean_plus_stddev_complete[i], alpha=0.5, color=c)
    axes[0].grid()
    axes[1].grid()

    plt.show()
elif(evaluation == 'recover'):
    bag = rosbag.Bag("/home/severin/luca_ws/rosbags/" + evaluation + "_evaluation/" + rosbags[0] + ".bag")

    counter = 1
    traj_pos_x = []
    traj_pos_z = []
    for topic, msg, t in bag.read_messages(topics='/firefly/command/trajectory'):
        if(counter == 1):
            starting_time = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
        elif(counter > time_tot * 10):
            break
        temp_traj_pos_x = []
        temp_traj_pos_z = []
        for point_msg in msg.points:
            temp_traj_pos_x.append(point_msg.transforms[0].translation.x)
            temp_traj_pos_z.append(point_msg.transforms[0].translation.z)
        traj_pos_x.append(temp_traj_pos_x)
        traj_pos_z.append(temp_traj_pos_z)
        counter += 1
    del(traj_pos_x[0])
    del(traj_pos_z[0])

    counter = 1
    uav_poses = []
    uav_pos_x = []
    uav_pos_z = []
    for topic, msg, t in bag.read_messages(topics='/firefly/odometry_sensor1/odometry'):
        current_time = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
        if(current_time < starting_time):
            continue
        elif(counter > time_tot * 100):
            break
        uav_pos_x.append(msg.pose.pose.position.x)
        uav_pos_z.append(msg.pose.pose.position.z)
        uav_poses.append(msg.pose.pose)
        counter += 1

    counter = 1
    target_positions = []
    target_pos_x = []
    target_pos_z = []
    for topic, msg, t in bag.read_messages(topics='/victim_drone/odometry'):
        current_time = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
        if(current_time < starting_time):
            continue
        elif(counter > time_tot * 100):
            break
        target_pos_x.append(msg.pose.pose.position.x)
        target_pos_z.append(msg.pose.pose.position.z)
        target_positions.append(msg.pose.pose.position)
        counter += 1

    fig = plt.figure()
    ax1 = fig.add_subplot(2, 2, 1)
    ax2 = fig.add_subplot(2, 2, 3)
    ax3 = fig.add_subplot(1, 2, 2)
    time = np.linspace(0, time_tot - 0.01, time_tot * 100)

    ax1.plot(time, target_pos_x, c='r')
    ax2.plot(time, target_pos_z, c='r')

    mode = "follow"
    for i in range(len(traj_pos_x)):
        traj_start_time = 0.1 * i
        time = np.linspace(traj_start_time, traj_start_time + 2.0, 201)
        if(traj_pos_z[i][len(traj_pos_z[i]) - 1] < traj_pos_z[i][0] and mode == "follow"):
            sw2rec = traj_start_time
            mode = "recover"
        elif(traj_pos_z[i][len(traj_pos_z[i]) - 1] >= traj_pos_z[i][0] and mode == "recover"):
            sw2foll = traj_start_time + 0.1
            mode = "follow"
        if(mode == "follow"):
            color = 'c'
        else:
            color = 'b'
        ax1.plot(time, traj_pos_x[i], c=color)
        ax2.plot(time, traj_pos_z[i], c=color)

    T_B_W = []
    for uav_pose in uav_poses:
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
    for target_position in target_positions:
        target_W.append(np.array([target_position.x, target_position.y, target_position.z, 1]))
    u = []
    v = []
    for j in range(0, len(T_B_W)):
        target_C = T_C_B.dot(T_B_W[j].dot(target_W[j]))
        u_v = K.dot(target_C)
        u.append(u_v[0] / u_v[2])
        v.append(u_v[1] / u_v[2])

    time1 = np.linspace(0, sw2rec - 0.01, sw2rec * 100)
    time2 = np.linspace(sw2rec, sw2foll - 0.01, (sw2foll - sw2rec) * 100)
    time3 = np.linspace(sw2foll, time_tot - 0.01, (time_tot - sw2foll) * 100)
    u1 = u[0 : int(sw2rec*100)]
    u2 = u[int(sw2rec*100) : int(sw2foll*100)]
    u3 = u[int(sw2foll*100) : int(time_tot*100)]
    ax3.plot(time1, u1, c='c')
    ax3.plot(time2, u2, c='b')
    ax3.plot(time3, u3, c='c')

    ax1.grid()
    ax2.grid()
    ax3.grid()

    plt.show()

# /firefly/command/trajectory /firefly/odometry_sensor1/odometry /victim_drone/odometry /firefly/target_detection /firefly/noise
