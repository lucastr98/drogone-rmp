import matplotlib.pyplot as plt
import matplotlib.animation as animation
import rosbag
import math
import numpy as np
import time
import matplotlib.patches as mpatches

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

bag = rosbag.Bag("/home/severin/luca_ws/rosbags/recover_evaluation/test9.bag")
time_tot = 14.0

counter = 0
recover_sw = []
back2follow_sw = []
for topic, msg, t in bag.read_messages(topics='/firefly/command/trajectory'):
    if counter == 0:
        starting_time = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
    if msg.points[0].accelerations[0].linear.z < -2.0:
        recover_sw.append(counter * 10)
    elif msg.points[0].accelerations[0].linear.z > 4.0:
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
last_x = 0
store_vel_sw = []
for topic, msg, t in bag.read_messages(topics='/victim_drone/odometry'):
    current_time = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
    if msg.pose.pose.position.x - last_x > 0.03 and msg.pose.pose.position.x - last_x < 0.05:
        store_vel_sw.append(counter)
    if(current_time < starting_time):
        continue
    elif(counter > time_tot * 100):
        break
    target_positions.append(msg.pose.pose.position)
    last_x = msg.pose.pose.position.x
    counter += 1

xy_distances = []
for i in range(len(uav_poses)):
    xy_distances.append(math.sqrt((uav_poses[i].position.x - target_positions[i].x) ** 2 + (uav_poses[i].position.y - target_positions[i].y) ** 2))

fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)
# ax1 = fig.add_subplot(2, 1, 1)
# ax2 = fig.add_subplot(2, 1, 2)
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
xy_distances_1 = xy_distances[0 : recover_sw[0]]
xy_distances_2 = xy_distances[recover_sw[0] : back2follow_sw[0]]
xy_distances_3 = xy_distances[back2follow_sw[0] : len(xy_distances)]
time_1 = time[0 : recover_sw[0]]
time_2 = time[recover_sw[0] : back2follow_sw[0]]
time_3 = time[back2follow_sw[0] : len(time)]

len_1 = len(u_1_1)
len_2 = len(u_1_1) + len(u_1_2)
len_3 = len(u_1_1) + len(u_1_2) + len(u_1_3)

def animate(i):
    if(i < len_1):
        ax1.plot(time_1[i], u_1_1[i], '.', c='b')
        # ax1.plot(time_1[i], xy_distances_1[i], '.', c='b')
        if(i == store_vel_sw[0] - 1):
            ax1.axvline(time_1[i], -200, 800, c='c')
        if(len(str(i)) == 1):
            fig.savefig('plot-000' + str(i) + '.png')
        elif(len(str(i)) == 2):
            fig.savefig('plot-00' + str(i) + '.png')
        elif(len(str(i)) == 3):
            fig.savefig('plot-0' + str(i) + '.png')
        elif(len(str(i)) == 4):
            fig.savefig('plot-' + str(i) + '.png')
    elif(i >= len_1 and i < len_2):
        ax1.plot(time_2[i - len_1], u_1_2[i - len_1], '.', c='r')
        # ax1.plot(time_2[i - len_1], xy_distances_2[i - len_1], '.', c='r')
        if(len(str(i)) == 1):
            fig.savefig('plot-000' + str(i) + '.png')
        elif(len(str(i)) == 2):
            fig.savefig('plot-00' + str(i) + '.png')
        elif(len(str(i)) == 3):
            fig.savefig('plot-0' + str(i) + '.png')
        elif(len(str(i)) == 4):
            fig.savefig('plot-' + str(i) + '.png')
    elif(i >= len_2 and i < len_3):
        ax1.plot(time_3[i - len_2], u_1_3[i - len_2], '.', c='b')
        # ax1.plot(time_3[i - len_2], xy_distances_3[i - len_2], '.', c='b')
        if(len(str(i)) == 1):
            fig.savefig('plot-000' + str(i) + '.png')
        elif(len(str(i)) == 2):
            fig.savefig('plot-00' + str(i) + '.png')
        elif(len(str(i)) == 3):
            fig.savefig('plot-0' + str(i) + '.png')
        elif(len(str(i)) == 4):
            fig.savefig('plot-' + str(i) + '.png')
    else:
        print("finished")

# ax1.set_ylim(0, 5)
# ax1.set_xlabel("time [s]")
# ax1.set_ylabel("horizontal distance [m]")
# ax1.set_title("horizontal distance from target to following UAV")
ax1.set_ylim(-300, 800)
ax1.set_xlabel("time [s]")
ax1.set_ylabel("u [px]")
ax1.set_title("u pixel of the camera image")
ax1.grid()
# handles = []
# handles.append(mpatches.Patch(color='b', label='Follow Mode'))
# handles.append(mpatches.Patch(color='r', label='Recover Mode'))
# handles.append(mpatches.Patch(hatch='|', color='c', label=r'$2 \frac{m}{s} \rightarrow 4 \frac{m}{s}$'))
# ax1.legend(handles=handles)

helper_ax = ax1.twinx()
ticks = []
labels = []
for i in np.linspace(0, 30, 4):
    ticks.append(math.tan(float(i) / 180.0 * np.pi) * 1140)
    labels.append(str(i))
helper_ax.set_yticks(ticks)
helper_ax.set_yticklabels(labels)
helper_ax.set_ylabel("angle [deg]")
helper_ax.set_ylim(-300, 800)

ani = animation.FuncAnimation(fig, animate, interval=10)

# mng = plt.get_current_fig_manager()
# mng.resize(*mng.window.maxsize())

plt.show()
