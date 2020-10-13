import rosbag
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.pyplot import cm
import math

plt.rcParams.update({'font.size': 15})

################################################################################
############################## FOR USER TO CHANGE ##############################
################################################################################

# fill in correct rosbag
bag = rosbag.Bag('/home/severin/luca_ws/rosbags/follow_evaluation/vel_two/linear/px_err_400_noise/test0.bag')

################################################################################
################################################################################
################################################################################

fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)

horizontal_acc_list = []
trajectory_time_list = []
counter = 0
for topic, msg, t in bag.read_messages(topics='/firefly/command/trajectory'):
    counter += 1
    if counter < 150 or counter > 160:
        continue
    number_of_points = len(msg.points)
    horizontal_acc = []
    trajectory_time = []
    for i in range(number_of_points):
        horizontal_acc.append(math.sqrt(msg.points[i].accelerations[0].linear.x**2 + msg.points[i].accelerations[0].linear.y**2))
        trajectory_time.append(msg.points[i].time_from_start.secs +
                               msg.points[i].time_from_start.nsecs/1e9 +
                               msg.header.stamp.secs +
                               msg.header.stamp.nsecs/1e9)
    horizontal_acc_list.append(horizontal_acc)
    trajectory_time_list.append(trajectory_time)

start_time = trajectory_time_list[0][0]
for i in range(0, len(trajectory_time_list)):
    for j in range(0, len(trajectory_time_list[i])):
        trajectory_time_list[i][j] -= start_time

number_of_traj = len(horizontal_acc_list)
color = iter(cm.rainbow(np.linspace(0, 1, number_of_traj)))
for i in range(0, number_of_traj, 1):
    c = next(color)
    ax1.plot(trajectory_time_list[i], horizontal_acc_list[i], '-', c=c)

ax1.grid()
ax1.set_xlabel('time [s]')
ax1.set_ylabel(r'acceleration $\left[ \frac{m}{s^2} \right]$')
ax1.set_title('horizontal acceleration of trajectory')

plt.show()
