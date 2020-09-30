import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import rosbag

bag = rosbag.Bag("/home/severin/luca_ws/rosbags/distance2ground_evaluation/test5.bag")

altitude = []
time = []
for topic, msg, t in bag.read_messages(topics='/altitude_node/intersection'):
    if msg.header.stamp.secs + msg.header.stamp.nsecs/1e9 < 39.0:
        continue
    time.append(msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)
    altitude.append(msg.point.z)

d2g_times = []
for topic, msg, t in bag.read_messages(topics='/firefly/command/trajectory'):
    if msg.points[0].accelerations[0].linear.x == 0 and msg.points[0].accelerations[0].linear.x == 0:
        d2g_times.append(msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)

time_1 = []
time_2 = []
time_3 = []
altitude_1 = []
altitude_2 = []
altitude_3 = []
for i in range(len(time)):
    if time[i] < d2g_times[0]:
        time_1.append(time[i])
        altitude_1.append(altitude[i])
    elif time[i] >= d2g_times[0] and time[i] < d2g_times[len(d2g_times) - 1]:
        time_2.append(time[i])
        altitude_2.append(altitude[i])
    elif time[i] >= d2g_times[len(d2g_times) - 1]:
        time_3.append(time[i])
        altitude_3.append(altitude[i])

start = time_1[0]
for i in range(len(time_1)):
    time_1[i] -= start
for i in range(len(time_2)):
    time_2[i] -= start
for i in range(len(time_3)):
    time_3[i] -= start

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)

len_1 = len(time_1)
len_2 = len(time_1) + len(time_2)
len_3 = len(time_1) + len(time_2) + len(time_3)
def animate(i):
    if(i < len_1):
        ax.plot(time_1[i], altitude_1[i], '.', c='r')
        if(len(str(i)) == 1):
            fig.savefig('plot-000' + str(i) + '.png')
        elif(len(str(i)) == 2):
            fig.savefig('plot-00' + str(i) + '.png')
        elif(len(str(i)) == 3):
            fig.savefig('plot-0' + str(i) + '.png')
        elif(len(str(i)) == 4):
            fig.savefig('plot-' + str(i) + '.png')
    elif(i >= len_1 and i < len_2):
        ax.plot(time_2[i - len_1], altitude_2[i - len_1], '.', c='orange')
        if(len(str(i)) == 1):
            fig.savefig('plot-000' + str(i) + '.png')
        elif(len(str(i)) == 2):
            fig.savefig('plot-00' + str(i) + '.png')
        elif(len(str(i)) == 3):
            fig.savefig('plot-0' + str(i) + '.png')
        elif(len(str(i)) == 4):
            fig.savefig('plot-' + str(i) + '.png')
    elif(i >= len_2 and i < len_3):
        ax.plot(time_3[i - len_2], altitude_3[i - len_2], '.', c='r')
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


ax.set_ylim(0, 8)
ax.set_xlabel("time [s]")
ax.set_ylabel("altitude [m]")
ax.set_title("altitude of UAV")
ax.grid()

ani = animation.FuncAnimation(fig, animate, interval=10)

plt.show()
