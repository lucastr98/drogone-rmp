import numpy as np
import matplotlib.pyplot as plt
import rosbag
import math
import matplotlib.patches as mpatches

time_tot = 20.0

bags = []
for i in range(15):
    bags.append(rosbag.Bag("/home/severin/luca_ws/rosbags/follow_evaluation/vel_two/linear/px_err_200_noise/test" + str(i) + ".bag"))
    bags.append(rosbag.Bag("/home/severin/luca_ws/rosbags/follow_evaluation/vel_two/linear/px_err_400_noise/test" + str(i) + ".bag"))
    bags.append(rosbag.Bag("/home/severin/luca_ws/rosbags/follow_evaluation/vel_two/linear/px_err_600_noise/test" + str(i) + ".bag"))
for i in range(15):
    bags.append(rosbag.Bag("/home/severin/luca_ws/rosbags/follow_evaluation/vel_two/random/px_err_200_noise/test" + str(i) + ".bag"))
    bags.append(rosbag.Bag("/home/severin/luca_ws/rosbags/follow_evaluation/vel_two/random/px_err_400_noise/test" + str(i) + ".bag"))
    bags.append(rosbag.Bag("/home/severin/luca_ws/rosbags/follow_evaluation/vel_two/random/px_err_600_noise/test" + str(i) + ".bag"))
for i in range(15):
    bags.append(rosbag.Bag("/home/severin/luca_ws/rosbags/follow_evaluation/vel_four/linear/px_err_200_noise/test" + str(i) + ".bag"))
    bags.append(rosbag.Bag("/home/severin/luca_ws/rosbags/follow_evaluation/vel_four/linear/px_err_400_noise/test" + str(i) + ".bag"))
    bags.append(rosbag.Bag("/home/severin/luca_ws/rosbags/follow_evaluation/vel_four/linear/px_err_600_noise/test" + str(i) + ".bag"))
for i in range(15):
    bags.append(rosbag.Bag("/home/severin/luca_ws/rosbags/follow_evaluation/vel_four/random/px_err_200_noise/test" + str(i) + ".bag"))
    bags.append(rosbag.Bag("/home/severin/luca_ws/rosbags/follow_evaluation/vel_four/random/px_err_400_noise/test" + str(i) + ".bag"))

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
for bag in bags:
    for topic, msg, t in bag.read_messages(topics='/firefly/command/trajectory'):
        starting_time = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
        break

    starting_time -= 0.25

    counter = 1
    for topic, msg, t in bag.read_messages(topics='/firefly/odometry_sensor1/odometry'):
        current_time = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
        if(current_time < starting_time):
            continue
        elif(counter > time_tot * 100):
            break
        uav_poses.append(msg.pose.pose)
        counter += 1

    counter = 1
    for topic, msg, t in bag.read_messages(topics='/victim_drone/odometry'):
        current_time = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
        if(current_time < starting_time):
            continue
        elif(counter > time_tot * 100):
            break
        target_positions.append(msg.pose.pose.position)
        counter += 1

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
ax1 = fig.add_subplot(1, 1, 1)

grid = 4
u_lim = 2048 + 208
v_lim = 1536 + 160
final = np.zeros((v_lim / grid, u_lim / grid))
for i in range(0, len(u)):
    # ax1.plot(u[i], v[i], '.', alpha=0.3, c='k')
    u_ix = int((u[i] + u_lim / 2) / grid)
    v_ix = int((v[i] + v_lim / 2) / grid)
    if v_ix >= int(v_lim / grid):
        # print(v[i])
        continue
    elif u_ix >= int(u_lim / grid):
        # print(u[i])
        continue
    elif v_ix < 0 or u_ix < 0:
        # print(v[i], u[i])
        continue
    final[v_ix, u_ix] += 1
the_max = np.amax(final) / 2
num_rows, num_cols = final.shape
for i in range(0, num_rows):
    print(i)
    for j in range(0, num_cols):
        x = [j * grid - u_lim / 2, j * grid - u_lim / 2 + grid, j * grid - u_lim / 2 + grid, j * grid - u_lim / 2]
        y = [i * grid - v_lim / 2, i * grid - v_lim / 2, i * grid - v_lim / 2 + grid, i * grid - v_lim / 2 + grid]
        importance = final[i, j] / the_max
        if importance > 1:
            importance = 1
        ax1.fill(x, y, 'k', alpha=importance)

point1 = [-1024, -768]
point2 = [-1024, 768]
point3 = [1024, 768]
point4 = [1024, -768]
x_values = [point1[0], point2[0], point3[0], point4[0], point1[0]]
y_values = [point1[1], point2[1], point3[1], point4[1], point1[1]]
ax1.plot(x_values, y_values, c='r', linewidth=2)

ax1.set_xlim(-u_lim / 2, u_lim / 2)
ax1.set_ylim(-v_lim / 2, v_lim / 2)
ax1.set_xlabel("u [px]")
ax1.set_ylabel("v [px]")
ax1.set_title("camera image with all follow mode evaluation points")
ax1.grid()

plt.show()
