import matplotlib.pyplot as plt
import matplotlib.animation as animation
import rosbag
import math
import numpy as np
import time
from matplotlib.pyplot import cm
from matplotlib import gridspec
from matplotlib.backends.backend_pdf import PdfPages

plt.rcParams.update({'font.size': 10})

bags = []
for i in range(15):
    bags.append(rosbag.Bag("/home/severin/luca_ws/rosbags/follow_evaluation/vel_four/random/px_err_400_noise/test" + str(i) + ".bag"))
for i in range(10):
    bags.append(rosbag.Bag("/home/severin/luca_ws/rosbags/follow_evaluation/vel_four/random/px_err_400_noise/fail" + str(i) + ".bag"))

time_tot = 20

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


uav_poses = []
target_positions = []
last_traj_time = []
for bag in bags:
    counter = 0
    for topic, msg, t in bag.read_messages(topics='/firefly/command/trajectory'):
        if counter == 0:
            starting_time = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
            counter = 1
        temp_last_traj_time = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9 - starting_time
    last_traj_time.append(temp_last_traj_time)

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
T_B_W_complete = []
target_W_complete = []
for i in range(0, len(uav_poses)):
    T_B_W = []
    for uav_pose in uav_poses[i]:
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
    T_B_W_complete.append(T_B_W)
    target_W = []
    for target_position in target_positions[i]:
        target_W.append(np.array([target_position.x, target_position.y, target_position.z, 1]))
    target_W_complete.append(target_W)
while len(T_B_W_complete[i]) > len(target_W_complete[i]):
    T_B_W_complete[i].pop()
while len(T_B_W_complete[i]) < len(target_W_complete[i]):
    target_W_complete[i].pop()
u_complete = []
v_complete = []
for i in range(len(T_B_W_complete)):
    u = []
    v = []
    for j in range(len(T_B_W_complete[i])):
        # print(i, j, len(T_B_W_complete[i]), len(target_W_complete[i]))
        target_C = T_C_B.dot(T_B_W_complete[i][j].dot(target_W_complete[i][j]))
        u_v = K.dot(target_C)
        if j * 0.01 > last_traj_time[i] and (abs(u_v[0] / u_v[2]) > 1024 or abs(u_v[1] / u_v[2]) > 768):
            break
        u.append(u_v[0] / u_v[2])
        v.append(u_v[1] / u_v[2])
    u_complete.append(u)
    v_complete.append(v)

pp = PdfPages('4_ran_400.pdf')

for i in range(len(bags)):
    fig = plt.figure()
    gs = gridspec.GridSpec(1, 2, width_ratios=[7, 1])
    ax1 = fig.add_subplot(gs[0])
    ax2 = fig.add_subplot(gs[1])

    success = 'successful'
    if i >= 15:
        success = 'fail'

    ax1.set_xlim(-1024, 1024)
    ax1.set_ylim(-768, 768)
    ax1.set_xlabel("u [px]")
    ax1.set_ylabel("v [px]")
    ax1.set_title(r"$4 \frac{m}{s}$, random, 400px, " + success)
    ax1.grid()

    circle10 = plt.Circle((0, 0), math.tan(10.0 / 180.0 * np.pi) * 1140, color=[1, 0.6, 0.6], fill=False)
    ax1.add_patch(circle10)
    ax1.annotate('$10 deg$', xy=(math.sqrt(2) / 2 * math.tan(10.0 / 180.0 * np.pi) * 1140, math.sqrt(2) / 2 * math.tan(10.0 / 180.0 * np.pi) * 1140), ha='left', va='bottom', size=10, color=[1, 0.6, 0.6])
    circle20 = plt.Circle((0, 0), math.tan(20.0 / 180.0 * np.pi) * 1140, color=[1, 0.6, 0.6], fill=False)
    ax1.add_patch(circle20)
    ax1.annotate('$20 deg$', xy=(math.sqrt(2) / 2 * math.tan(20.0 / 180.0 * np.pi) * 1140, math.sqrt(2) / 2 * math.tan(20.0 / 180.0 * np.pi) * 1140), ha='left', va='bottom', size=10, color=[1, 0.6, 0.6])
    circle30 = plt.Circle((0, 0), math.tan(30.0 / 180.0 * np.pi) * 1140, color=[1, 0.6, 0.6], fill=False)
    ax1.add_patch(circle30)
    ax1.annotate('$30 deg$', xy=(math.sqrt(2) / 2 * math.tan(30.0 / 180.0 * np.pi) * 1140, math.sqrt(2) / 2 * math.tan(30.0 / 180.0 * np.pi) * 1140), ha='left', va='bottom', size=10, color=[1, 0.6, 0.6])
    circle40 = plt.Circle((0, 0), math.tan(40.0 / 180.0 * np.pi) * 1140, color=[1, 0.6, 0.6], fill=False)
    ax1.add_patch(circle40)
    ax1.annotate('$40 deg$', xy=(math.sqrt(2) / 2 * math.tan(40.0 / 180.0 * np.pi) * 1140, math.sqrt(2) / 2 * math.tan(40.0 / 180.0 * np.pi) * 1140), ha='left', va='bottom', size=10, color=[1, 0.6, 0.6])

    color = iter(cm.rainbow(np.linspace(0, 1, len(u_complete[i]))))
    for j in range(0, len(u_complete[i])):
        c = next(color)
        ax1.plot(u_complete[i][j], v_complete[i][j], '.', c=c)
        ax2.axhline(j * 0.01, color=c)

    ax2.set_ylim(0, len(u_complete[i]) * 0.01)
    ax2.set_xticklabels([])
    ax2.set_ylabel("time [s]")
    ax2.yaxis.set_label_position("right")
    ax2.yaxis.tick_right()

    plt.tight_layout()

    # mng = plt.get_current_fig_manager()
    # mng.resize(*mng.window.maxsize())

    pp.savefig(fig)
pp.close()
