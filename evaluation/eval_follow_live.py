import matplotlib.pyplot as plt
import matplotlib.animation as animation
import rosbag
import math
import numpy as np
import time
from matplotlib.pyplot import cm
from matplotlib import gridspec

plt.rcParams.update({'font.size': 15})

bag = rosbag.Bag("/home/severin/luca_ws/rosbags/follow_evaluation/presentation/test1.bag")

start_time = 65.0
end_time = 85.0
time_tot = end_time - start_time

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
for topic, msg, t in bag.read_messages(topics='/firefly/odometry_sensor1/odometry'):
    current_time = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
    if(current_time < start_time):
        continue
    elif(current_time > end_time):
        break
    uav_poses.append(msg.pose.pose)

target_positions = []
for topic, msg, t in bag.read_messages(topics='/victim_drone/odometry'):
    current_time = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
    if(current_time < start_time):
        continue
    elif(current_time > end_time):
        break
    if(current_time == 69.98):
        continue
    target_positions.append(msg.pose.pose.position)

T_B_W = []
for uav_pose in uav_poses:
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

fig = plt.figure()
gs = gridspec.GridSpec(1, 2, width_ratios=[5, 1])
ax1 = fig.add_subplot(gs[0])
ax2 = fig.add_subplot(gs[1])

# ax1 = fig.add_subplot(1, 1, 1)

ax1.set_xlim(-1024, 1024)
ax1.set_ylim(-768, 768)
ax1.set_xlabel("u [px]")
ax1.set_ylabel("v [px]")
ax1.set_title("camera image following a 2m/s randomly moving target")
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

# def animate(i):
#     if(i < len(u)):
#         ax1.plot(u[i], v[i], '.', c='b')
#         if(len(str(i)) == 1):
#             fig.savefig('plot-000' + str(i) + '.png')
#         elif(len(str(i)) == 2):
#             fig.savefig('plot-00' + str(i) + '.png')
#         elif(len(str(i)) == 3):
#             fig.savefig('plot-0' + str(i) + '.png')
#         elif(len(str(i)) == 4):
#             fig.savefig('plot-' + str(i) + '.png')
#     else:
#         print("finished")
#
#
#
# ani = animation.FuncAnimation(fig, animate, interval=10)

color = iter(cm.rainbow(np.linspace(0, 1, len(u))))
for i in range(0, len(u)):
    c = next(color)
    ax1.plot(u[i], v[i], '.', c=c)
    ax2.axhline(i * 0.01, color=c)

ax2.set_ylim(0, 20)
ax2.set_xticklabels([])
ax2.set_ylabel("time [s]")
ax2.yaxis.set_label_position("right")
ax2.yaxis.tick_right()
plt.show()

# mng = plt.get_current_fig_manager()
# mng.resize(*mng.window.maxsize())

# for i in range(len(pose_times)):
#     print(pose_times[i] == target_times[i], i)
# print(pose_times[387], target_times[387], "---", pose_times[388], target_times[388], "---", pose_times[389], target_times[389])
# print(pose_times[453], target_times[453], "---", pose_times[454], target_times[454], "---", pose_times[455], target_times[455], "---", pose_times[456], target_times[456])
# print(pose_times[866], target_times[866], "---", pose_times[867], target_times[867], "---", pose_times[868], target_times[868], "---", pose_times[869], target_times[869])
# print(pose_times[1039], target_times[1039], "---", pose_times[1040], target_times[1040], "---", pose_times[1041], target_times[1041], "---", pose_times[1042], target_times[1042])
# print(pose_times[1053], target_times[1053], "---", pose_times[1054], target_times[1054], "---", pose_times[1055], target_times[1055])
# print(pose_times[1082], target_times[1082], "---", pose_times[1083], target_times[1083], "---", pose_times[1084], target_times[1084])
# print(pose_times[1446], target_times[1446], "---", pose_times[1447], target_times[1447], "---", pose_times[1448], target_times[1448])
