import numpy as np
import matplotlib.pyplot as plt
from matplotlib.pyplot import cm
import rosbag
import yaml
import math
import matplotlib.patches as mpatches

plt.rcParams.update({'font.size': 15})

# load bags
with open(r'evaluation.yaml') as file:
    list = yaml.safe_load(file)
    evaluation = list['evaluation']
    target_velocity = list['target_velocity']
    flight_path = list['flight_path']
    success = list['success']
    init_px_err = list['init_px_err']
    rosbags = list['rosbags']
    catch_success = list['catch_success']
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
    bags_complete = []
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

            starting_time -= 0.25

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

    fig, axes = plt.subplots(2, 1)
    time = np.linspace(0, time_tot - 0.01, time_tot * 100)
    color = iter(cm.rainbow(np.linspace(0, 1, n)))

    handles = []
    for i in range(n):
        c = next(color)
        # if i == 0:
        #     c = [0.5, 0, 1, 1]
        # elif i == 1:
        #     c = [0.50392157, 0.99998103, 0.70492555, 1]
        axes[0].plot(time, px_mean_complete[i], c=c)
        axes[0].fill_between(time, px_mean_minus_stddev_complete[i], px_mean_plus_stddev_complete[i], alpha=0.5, color=c)
        axes[1].plot(time, xy_mean_complete[i], c=c)
        axes[1].fill_between(time, xy_mean_minus_stddev_complete[i], xy_mean_plus_stddev_complete[i], alpha=0.5, color=c)
        handles.append(mpatches.Patch(color=c, label='initial error: ' + str(init_px_err[i])[0:3] + 'px, success: 15/' + str(success[i])))
    axes[0].set_xlabel("time [s]")
    axes[0].set_ylabel("pixel error [px]")
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("horizontal error in world frame [m]")
    axes[0].grid()
    axes[1].grid()

    handles.reverse()
    axes[0].legend(handles=handles)
    axes[1].legend(handles=handles)

    helper_ax = axes[0].twinx()
    ticks = []
    labels = []
    for i in np.linspace(0, 50, 6):
        ticks.append(math.tan(float(i) / 180.0 * np.pi) * 1140)
        labels.append(str(i))
    helper_ax.set_yticks(ticks)
    helper_ax.set_yticklabels(labels)
    helper_ax.set_ylabel("angle [deg]")

    helper_ax.set_ylim(-50, 700)
    axes[0].set_ylim(-50, 700)

    vel = 0
    if target_velocity == 'two':
        vel = 2
    elif target_velocity == 'four':
        vel = 4
    fig.suptitle('follow mode, target flying ' + flight_path + 'ly with ' + str(vel) + r'$\frac{m}{s}$', fontsize=16, y=0.92)

    plt.show()

elif(evaluation == 'recover'):
    bag = rosbag.Bag("/home/severin/luca_ws/rosbags/" + evaluation + "_evaluation/" + rosbags[0] + ".bag")

    counter = 0
    recover_sw = []
    back2follow_sw = []
    for topic, msg, t in bag.read_messages(topics='/firefly/command/trajectory'):
        if counter == 0:
            starting_time = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
        if msg.points[0].accelerations[0].linear.z < -2.0 and len(back2follow_sw) == 0:
            recover_sw.append(counter * 10)
        elif msg.points[0].accelerations[0].linear.z > 4.0 and counter > 10:
            back2follow_sw.append(counter * 10)
        counter += 1

    counter = 1
    uav_poses = []
    for topic, msg, t in bag.read_messages(topics='/firefly/odometry_sensor1/odometry'):
        current_time = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
        if(current_time < starting_time):
            continue
        elif(counter > time_tot * 100):
            break
        uav_poses.append(msg.pose.pose)
        counter += 1

    counter = 1
    target_positions = []
    for topic, msg, t in bag.read_messages(topics='/victim_drone/odometry'):
        current_time = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
        if(current_time < starting_time):
            continue
        elif(counter > time_tot * 100):
            break
        target_positions.append(msg.pose.pose.position)
        counter += 1

    fig = plt.figure()
    ax1 = fig.add_subplot(2, 1, 1)
    ax2 = fig.add_subplot(2, 1, 2)
    time = np.linspace(0, time_tot - 0.01, time_tot * 100)

    T_B_W_1 = []
    T_B_W_2 = []
    for i in range(2):
        for uav_pose in uav_poses:
            if i == 0:
                roll_B_W = math.atan2(2.0 * (uav_pose.orientation.w * uav_pose.orientation.x + uav_pose.orientation.y * uav_pose.orientation.z),
                1.0 - 2.0 * (uav_pose.orientation.x ** 2 - uav_pose.orientation.y ** 2))
                pitch_B_W = 2.0 * (uav_pose.orientation.w * uav_pose.orientation.y - uav_pose.orientation.z * uav_pose.orientation.x)
            else:
                roll_B_W = 0.0
                pitch_B_W = 0.0
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
            if i == 0:
                T_B_W_1.append(temp_T_B_W)
            else:
                T_B_W_2.append(temp_T_B_W)

    target_W = []
    for target_position in target_positions:
        target_W.append(np.array([target_position.x, target_position.y, target_position.z, 1]))
    u_1 = []
    for j in range(0, len(T_B_W_1)):
        target_C = T_C_B.dot(T_B_W_1[j].dot(target_W[j]))
        u_v = K.dot(target_C)
        u_1.append(u_v[0] / u_v[2])
    u_2 = []
    for j in range(0, len(T_B_W_2)):
        target_C = T_C_B.dot(T_B_W_2[j].dot(target_W[j]))
        u_v = K.dot(target_C)
        u_2.append(u_v[0] / u_v[2])

    u_1_1 = u_1[0 : recover_sw[0]]
    u_1_2 = u_1[recover_sw[0] : back2follow_sw[0]]
    u_1_3 = u_1[back2follow_sw[0] : len(u_1)]
    u_2_1 = u_2[0 : recover_sw[0]]
    u_2_2 = u_2[recover_sw[0] : back2follow_sw[0]]
    u_2_3 = u_2[back2follow_sw[0] : len(u_2)]
    time_1 = time[0 : recover_sw[0]]
    time_2 = time[recover_sw[0] : back2follow_sw[0]]
    time_3 = time[back2follow_sw[0] : len(time)]

    ax1.plot(time_1, u_1_1, c='c')
    ax1.plot(time_2, u_1_2, c='b')
    ax1.plot(time_3, u_1_3, c='c')
    ax2.plot(time_1, u_2_1, c='c')
    ax2.plot(time_2, u_2_2, c='b')
    ax2.plot(time_3, u_2_3, c='c')

    # ax1.set_xlabel('time [s]')
    ax1.set_ylabel('u [px]')
    ax1.set_title('actual roll and pitch')
    ax2.set_xlabel('time [s]')
    ax2.set_ylabel('u [px]')
    ax2.set_title('roll and pitch zero')

    fig.suptitle('Recover Mode', y=0.95, fontweight='bold')

    handles = []
    handles.append(mpatches.Patch(color='c', label='Follow Mode'))
    handles.append(mpatches.Patch(color='b', label='Recover Mode'))
    ax1.legend(handles=handles)
    ax2.legend(handles=handles)


    ax1.grid()
    ax2.grid()

    plt.show()

elif(evaluation == 'catch'):
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)

    path = ['stationary', r'2 $\frac{m}{s}$ linear', r'2 $\frac{m}{s}$ random', r'4 $\frac{m}{s}$ linear', r'4 $\frac{m}{s}$ random']
    success = [catch_success[0], catch_success[1], catch_success[2], catch_success[3], catch_success[4]]
    ax.bar(path, success)
    ax.set_ylabel("successful catches")
    ax.set_title("successful catches for different target flight maneouvres")
    ax.yaxis.grid()
    ticks = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
    ax.set_yticks(ticks)
    ax.set_axisbelow(True)

    plt.show()

elif(evaluation == 'noise'):
    px_err_val = init_px_err[0]
    bag = rosbag.Bag("/home/severin/luca_ws/rosbags/follow_evaluation/vel_" + target_velocity + "/" + flight_path + "/px_err_" + str(px_err_val) + "/" + rosbags[0] + ".bag")

    x_C_noise = []
    y_C_noise = []
    z_C_noise = []
    z_C = []
    time = []
    for topic, msg, t in bag.read_messages(topics='/firefly/noise'):
        time.append(msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)
        x_C_noise.append(float(msg.noise_x_C) / msg.z_C * 1140)
        y_C_noise.append(float(msg.noise_y_C) / msg.z_C * 1140)
        z_C_noise.append(msg.noise_z_C)
        z_C.append(msg.z_C)

    start_time = time[0]
    for i in range(len(time)):
        time[i] -= start_time

    fig = plt.figure()

    ax1 = fig.add_subplot(3, 1, 1)
    ax2 = fig.add_subplot(3, 1, 2)
    ax3 = fig.add_subplot(3, 1, 3)

    ax1.plot(time, x_C_noise, c='r')
    ax1.plot(time, y_C_noise, c='g')
    ax2.plot(time, z_C_noise, c='b')
    ax3.plot(time, z_C, c='k')

    handles = []
    handles.append(mpatches.Patch(color='r', label='$x_C$'))
    handles.append(mpatches.Patch(color='g', label='$y_C$'))
    ax1.legend(handles=handles)
    handles = []
    handles.append(mpatches.Patch(color='b', label='$z_C$'))
    ax2.legend(handles=handles)

    ax3.set_xlabel('time [s]')
    ax1.set_ylabel('noise [px]')
    ax2.set_ylabel('noise [m]')
    ax3.set_ylabel("$z_C$ [m]")

    ax1.set_title('example noise for evaluation')

    ax1.grid()
    ax2.grid()
    ax3.grid()

    plt.show()

elif(evaluation == 'distance2ground'):
    bag1 = rosbag.Bag("/home/severin/luca_ws/rosbags/" + evaluation + "_evaluation/" + rosbags[0] + ".bag")     # test5
    bag2 = rosbag.Bag("/home/severin/luca_ws/rosbags/" + evaluation + "_evaluation/" + rosbags[1] + ".bag")     # new_test0

    altitude_1 = []
    time_1 = []
    for topic, msg, t in bag1.read_messages(topics='/altitude_node/intersection'):
        if msg.header.stamp.secs + msg.header.stamp.nsecs/1e9 < 39.0:
            continue
        time_1.append(msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)
        altitude_1.append(msg.point.z)

    time_2_help = []
    for topic, msg, t in bag2.read_messages(topics='/altitude_node/intersection'):
        if msg.header.stamp.secs + msg.header.stamp.nsecs/1e9 < 26.0:
            continue
        time_2_help.append(msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)
        if time_2_help[len(time_2_help) - 1] == 31.07:
            for i in range(50):
                time_2_help.append(round(31.07 + (i + 1) * 0.02, 2))

    time_2 = []
    altitude_2 = []
    for topic, msg, t in bag2.read_messages(topics='/firefly/odometry_sensor1/odometry'):
        if msg.header.stamp.secs + msg.header.stamp.nsecs/1e9 == time_2_help[0]:
            altitude_2.append(msg.pose.pose.position.z)
            time_2.append(time_2_help[0])
            time_2_help.pop(0)

    d2g_times_1 = []
    for topic, msg, t in bag1.read_messages(topics='/firefly/command/trajectory'):
        if msg.points[0].accelerations[0].linear.x == 0 and msg.points[0].accelerations[0].linear.x == 0:
            d2g_times_1.append(msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)

    d2g_times_2_1 = []
    d2g_times_2_2 = []
    for topic, msg, t in bag2.read_messages(topics='/firefly/command/trajectory'):
        if msg.points[0].accelerations[0].linear.x == 0 and msg.points[0].accelerations[0].linear.x == 0:
            if len(d2g_times_2_1) == 0:
                d2g_times_2_1.append(msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)
            elif msg.header.stamp.secs + msg.header.stamp.nsecs/1e9 - d2g_times_2_1[len(d2g_times_2_1) - 1] > 1.0:
                d2g_times_2_2.append(msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)
            else:
                d2g_times_2_1.append(msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)

    time_1_1 = []
    time_1_2 = []
    time_1_3 = []
    altitude_1_1 = []
    altitude_1_2 = []
    altitude_1_3 = []
    for i in range(len(time_1)):
        if time_1[i] < d2g_times_1[0]:
            time_1_1.append(time_1[i])
            altitude_1_1.append(altitude_1[i])
        elif time_1[i] >= d2g_times_1[0] and time_1[i] < d2g_times_1[len(d2g_times_1) - 1]:
            time_1_2.append(time_1[i])
            altitude_1_2.append(altitude_1[i])
        elif time_1[i] >= d2g_times_1[len(d2g_times_1) - 1]:
            time_1_3.append(time_1[i])
            altitude_1_3.append(altitude_1[i])
    start = time_1_1[0]
    for i in range(len(time_1_1)):
        time_1_1[i] -= start
    for i in range(len(time_1_2)):
        time_1_2[i] -= start
    for i in range(len(time_1_3)):
        time_1_3[i] -= start

    time_2_1 = []
    time_2_2 = []
    time_2_3 = []
    time_2_4 = []
    time_2_5 = []
    altitude_2_1 = []
    altitude_2_2 = []
    altitude_2_3 = []
    altitude_2_4 = []
    altitude_2_5 = []
    for i in range(len(time_2)):
        if time_2[i] < d2g_times_2_1[0]:
            time_2_1.append(time_2[i])
            altitude_2_1.append(altitude_2[i])
        elif time_2[i] >= d2g_times_2_1[0] and time_2[i] < d2g_times_2_1[len(d2g_times_2_1) - 1]:
            time_2_2.append(time_2[i])
            altitude_2_2.append(altitude_2[i])
        elif time_2[i] >= d2g_times_2_1[len(d2g_times_2_1) - 1] and time_2[i] < d2g_times_2_2[0]:
            time_2_3.append(time_2[i])
            altitude_2_3.append(altitude_2[i])
        elif time_2[i] >= d2g_times_2_2[0] and time_2[i] < d2g_times_2_2[len(d2g_times_2_2) - 1]:
            time_2_4.append(time_2[i])
            altitude_2_4.append(altitude_2[i])
        elif time_2[i] >= d2g_times_2_2[len(d2g_times_2_2) - 1]:
            time_2_5.append(time_2[i])
            altitude_2_5.append(altitude_2[i])
    start = time_2_1[0]
    for i in range(len(time_2_1)):
        time_2_1[i] -= start
    for i in range(len(time_2_2)):
        time_2_2[i] -= start
    for i in range(len(time_2_3)):
        time_2_3[i] -= start
    for i in range(len(time_2_4)):
        time_2_4[i] -= start
    for i in range(len(time_2_5)):
        time_2_5[i] -= start

    fig = plt.figure()
    ax1 = fig.add_subplot(1, 2, 1)
    ax2 = fig.add_subplot(1, 2, 2)

    ax1.plot(time_1_1, altitude_1_1, c='c')
    ax1.plot(time_1_2, altitude_1_2, c='b')
    ax1.plot(time_1_3, altitude_1_3, c='c')

    ax2.plot(time_2_1, altitude_2_1, c='c')
    ax2.plot(time_2_2, altitude_2_2, c='b')
    ax2.plot(time_2_3, altitude_2_3, c='c')
    ax2.plot(time_2_4, altitude_2_4, c='b')
    ax2.plot(time_2_5, altitude_2_5, c='c')

    handles = []
    handles.append(mpatches.Patch(color='c', label='Follow Mode'))
    handles.append(mpatches.Patch(color='b', label='Prevent Ground Crash Mode'))
    ax1.legend(handles=handles)
    ax2.legend(handles=handles)

    ax1.set_ylim(0, 7.5)
    ax1.set_xlabel("time [s]")
    ax1.set_ylabel("altitude [m]")
    ax1.set_title("Mountain Experiment")
    ax1.grid()
    ax2.set_ylim(0, 7.5)
    ax2.set_xlabel("time [s]")
    ax2.set_ylabel("altitude [m]")
    ax2.set_title("Ground Experiment")
    ax2.grid()

    plt.show()

elif(evaluation == 'recover_distance'):
    bag = rosbag.Bag("/home/severin/luca_ws/rosbags/" + evaluation + "_evaluation/" + rosbags[0] + ".bag")     # test0

    counter = 0
    for topic, msg, t in bag.read_messages(topics='/firefly/command/trajectory'):
        if counter == 0:
            starting_time = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
        counter += 1

    counter = 1
    uav_positions = []
    time = []
    for topic, msg, t in bag.read_messages(topics='/firefly/odometry_sensor1/odometry'):
        current_time = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
        if(current_time < starting_time):
            continue
        elif(counter > time_tot * 100):
            break
        uav_positions.append(msg.pose.pose.position)
        time.append(current_time - starting_time)
        counter += 1

    counter = 1
    target_positions = []
    for topic, msg, t in bag.read_messages(topics='/victim_drone/odometry'):
        current_time = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
        if(current_time < starting_time):
            continue
        elif(counter > time_tot * 100):
            break
        target_positions.append(msg.pose.pose.position)
        counter += 1

    distances = []
    for i in range(0, len(uav_positions)):
        distances.append(math.sqrt((target_positions[i].x - uav_positions[i].x)**2 +
                                   (target_positions[i].y - uav_positions[i].y)**2 +
                                   (target_positions[i].z - uav_positions[i].z)**2))

    distances_1 = []
    distances_2 = []
    distances_3 = []
    time_1 = []
    time_2 = []
    time_3 = []
    for i in range(0, len(distances)):
        if distances[i] < 17 and len(distances_2) == 0:
            distances_1.append(distances[i])
            time_1.append(time[i])
        elif distances[i] >= 17:
            distances_2.append(distances[i])
            time_2.append(time[i])
        elif distances[i] < 17 and distances[i] > 12 and len(distances_2) > 0 and len(distances_3) == 0:
            distances_2.append(distances[i])
            time_2.append(time[i])
        else:
            distances_3.append(distances[i])
            time_3.append(time[i])

    fig = plt.figure()
    ax1 = fig.add_subplot(1, 1, 1)

    ax1.plot(time_1, distances_1, c='c')
    ax1.plot(time_2, distances_2, c='b')
    ax1.plot(time_3, distances_3, c='c')

    handles = []
    handles.append(mpatches.Patch(color='c', label='Follow Mode'))
    handles.append(mpatches.Patch(color='b', label='Recover Distance Mode'))
    ax1.legend(handles=handles)

    ax1.set_xlabel("time [s]")
    ax1.set_ylabel("distance to target [m]")
    ax1.set_title("Recover Distance Mode")
    ax1.grid()

    plt.show()

elif(evaluation == 'follow_distances'):
    bags_complete = []
    bags_1 = []
    bags_2 = []
    bags_3 = []
    bags_4 = []
    for i in range(15):
        bags_1.append(rosbag.Bag("/home/severin/luca_ws/rosbags/follow_evaluation/vel_two/linear/px_err_200_noise/test" + str(i) + ".bag"))
        bags_1.append(rosbag.Bag("/home/severin/luca_ws/rosbags/follow_evaluation/vel_two/linear/px_err_400_noise/test" + str(i) + ".bag"))
        bags_1.append(rosbag.Bag("/home/severin/luca_ws/rosbags/follow_evaluation/vel_two/linear/px_err_600_noise/test" + str(i) + ".bag"))
    bags_complete.append(bags_1)
    for i in range(15):
        bags_2.append(rosbag.Bag("/home/severin/luca_ws/rosbags/follow_evaluation/vel_two/random/px_err_200_noise/test" + str(i) + ".bag"))
        bags_2.append(rosbag.Bag("/home/severin/luca_ws/rosbags/follow_evaluation/vel_two/random/px_err_400_noise/test" + str(i) + ".bag"))
        bags_2.append(rosbag.Bag("/home/severin/luca_ws/rosbags/follow_evaluation/vel_two/random/px_err_600_noise/test" + str(i) + ".bag"))
    bags_complete.append(bags_2)
    for i in range(15):
        bags_3.append(rosbag.Bag("/home/severin/luca_ws/rosbags/follow_evaluation/vel_four/linear/px_err_200_noise/test" + str(i) + ".bag"))
        bags_3.append(rosbag.Bag("/home/severin/luca_ws/rosbags/follow_evaluation/vel_four/linear/px_err_400_noise/test" + str(i) + ".bag"))
        bags_3.append(rosbag.Bag("/home/severin/luca_ws/rosbags/follow_evaluation/vel_four/linear/px_err_600_noise/test" + str(i) + ".bag"))
    bags_complete.append(bags_3)
    for i in range(15):
        bags_4.append(rosbag.Bag("/home/severin/luca_ws/rosbags/follow_evaluation/vel_four/random/px_err_200_noise/test" + str(i) + ".bag"))
        bags_4.append(rosbag.Bag("/home/severin/luca_ws/rosbags/follow_evaluation/vel_four/random/px_err_400_noise/test" + str(i) + ".bag"))
    bags_complete.append(bags_4)

    uav_positions_complete = []
    target_positions_complete = []
    for bags in bags_complete:
        uav_positions = []
        target_positions = []
        for bag in bags:
            for topic, msg, t in bag.read_messages(topics='/firefly/command/trajectory'):
                starting_time = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
                break

            starting_time -= 0.25

            counter = 1
            temp_uav_positions = []
            for topic, msg, t in bag.read_messages(topics='/firefly/odometry_sensor1/odometry'):
                current_time = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
                if(current_time < starting_time):
                    continue
                elif(counter > time_tot * 100):
                    break
                temp_uav_positions.append(msg.pose.pose.position)
                counter += 1
            uav_positions.append(temp_uav_positions)

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
        uav_positions_complete.append(uav_positions)
        target_positions_complete.append(target_positions)

    distances_complete = []
    for i in range(0, len(uav_positions_complete)):
        distances = []
        for j in range(0, len(uav_positions_complete[i])):
            temp_distances = []
            for r in range(0, len(uav_positions_complete[i][j])):
                temp_distances.append(math.sqrt((uav_positions_complete[i][j][r].x - target_positions_complete[i][j][r].x)**2 +
                                                (uav_positions_complete[i][j][r].y - target_positions_complete[i][j][r].y)**2 +
                                                (uav_positions_complete[i][j][r].z - target_positions_complete[i][j][r].z)**2))
            distances.append(temp_distances)
        distances_complete.append(distances)

    mean_complete = []
    stddev_complete = []
    mean_plus_stddev_complete = []
    mean_minus_stddev_complete = []
    for i in range(0, len(distances_complete)):
        mean = []
        stddev = []
        mean_plus_stddev = []
        mean_minus_stddev = []
        for r in range(0, len(distances_complete[i][0])):
            sum = 0
            for j in range(0, len(distances_complete[i])):
                sum += distances_complete[i][j][r]
            mean.append(sum / len(distances_complete[i]))
            stddev_sum = 0
            for j in range(0, len(distances_complete[i])):
                stddev_sum += (distances_complete[i][j][r] -  mean[len(mean) - 1])**2
            stddev.append(math.sqrt(stddev_sum / len(distances_complete[i])))
            mean_plus_stddev.append(mean[len(mean) - 1] + stddev[len(stddev) - 1])
            mean_minus_stddev.append(mean[len(mean) - 1] - stddev[len(stddev) - 1])
        mean_complete.append(mean)
        stddev_complete.append(stddev)
        mean_plus_stddev_complete.append(mean_plus_stddev)
        mean_minus_stddev_complete.append(mean_minus_stddev)

    fig = plt.figure()
    ax1 = fig.add_subplot(2, 2, 1)
    ax2 = fig.add_subplot(2, 2, 2)
    ax3 = fig.add_subplot(2, 2, 3)
    ax4 = fig.add_subplot(2, 2, 4)
    time = np.linspace(0, time_tot - 0.01, time_tot * 100)

    ax1.plot(time, mean_complete[0])
    ax1.fill_between(time, mean_minus_stddev_complete[0], mean_plus_stddev_complete[0], alpha=0.5)
    ax2.plot(time, mean_complete[1])
    ax2.fill_between(time, mean_minus_stddev_complete[1], mean_plus_stddev_complete[1], alpha=0.5)
    ax3.plot(time, mean_complete[2])
    ax3.fill_between(time, mean_minus_stddev_complete[2], mean_plus_stddev_complete[2], alpha=0.5)
    ax4.plot(time, mean_complete[3])
    ax4.fill_between(time, mean_minus_stddev_complete[3], mean_plus_stddev_complete[3], alpha=0.5)

    ax3.set_xlabel('time [s]')
    ax4.set_xlabel('time [s]')
    ax1.set_ylim(3, 20)
    ax2.set_ylim(3, 20)
    ax3.set_ylim(3, 20)
    ax4.set_ylim(3, 20)
    ax1.set_ylabel('distance [m]')
    ax2.set_ylabel('distance [m]')
    ax3.set_ylabel('distance [m]')
    ax4.set_ylabel('distance [m]')
    ax1.set_title(r'$2 \frac{m}{s}$ linear')
    ax2.set_title(r'$2 \frac{m}{s}$ random')
    ax3.set_title(r'$4 \frac{m}{s}$ linear')
    ax4.set_title(r'$4 \frac{m}{s}$ random')
    ax1.grid()
    ax2.grid()
    ax3.grid()
    ax4.grid()

    plt.show()


# /firefly/command/trajectory /firefly/odometry_sensor1/odometry /victim_drone/odometry /firefly/target_detection /firefly/noise /altitude_node/intersection
