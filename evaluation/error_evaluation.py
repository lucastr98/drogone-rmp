import numpy as np
import matplotlib.pyplot as plt
import rosbag
import math

time_tot = 20

bag = rosbag.Bag("/home/severin/luca_ws/rosbags/follow_evaluation/vel_two/linear/px_err_400_noise/test0.bag")

pos_x = []
pos_y = []
pos_z = []
noise_x = []
noise_y = []
noise_z = []

last_pos_x = 0.0
last_pos_y = 0.0
counter = -1
started = False
for topic, msg, t in bag.read_messages(topics='/firefly/noise'):
    if(counter == -1):
        last_pos_x = msg.x_C
        last_pos_y = msg.y_C
        counter += 1
        continue

    cur_pos = math.sqrt(msg.x_C ** 2 + msg.y_C ** 2)
    last_pos = math.sqrt(last_pos_x ** 2 + last_pos_y ** 2)
    if((abs(cur_pos - last_pos) < 0.1) and started == False):
        continue
    else:
        started = True

    if(counter >= time_tot * 10):
        break
    pos_x.append(msg.x_C)
    pos_y.append(msg.y_C)
    pos_z.append(msg.z_C)
    noise_x.append(msg.x_C + msg.noise_x_C)
    noise_y.append(msg.y_C + msg.noise_y_C)
    noise_z.append(msg.z_C + msg.noise_z_C)
    last_pos_x = msg.x_C
    last_pos_y = msg.y_C
    counter += 1

fig, axes = plt.subplots(3, 1) #, sharex=True)
time = np.linspace(0, time_tot - 0.01, time_tot * 10)
axes[0].plot(time, pos_x, c='r', label='actual position')
axes[0].plot(time, noise_x, c='b', label='noisy position')
axes[0].set_xlabel("time [s]")
axes[0].set_ylabel("x camera frame [m]")
axes[0].grid()
axes[1].plot(time, pos_y, c='r', label='actual position')
axes[1].plot(time, noise_y, c='b', label='noisy position')
axes[1].set_xlabel("time [s]")
axes[1].set_ylabel("y camera frame [m]")
axes[1].grid()
axes[2].plot(time, pos_z, c='r', label='actual position')
axes[2].plot(time, noise_z, c='b', label='noisy position')
axes[2].set_xlabel("time [s]")
axes[2].set_ylabel("z camera frame [m]")
axes[2].grid()

plt.show()

    # pos_x_mean = []
    # pos_x_stddev = []
    # pos_x_mean_plus_stddev = []
    # pos_x_mean_minus_stddev = []
    # pos_y_mean = []
    # pos_y_stddev = []
    # pos_y_mean_plus_stddev = []
    # pos_y_mean_minus_stddev = []
    # pos_y_mean = []
    # pos_y_stddev = []
    # pos_y_mean_plus_stddev = []
    # pos_y_mean_minus_stddev = []
    # noise_x_mean = []
    # noise_x_stddev = []
    # noise_x_mean_plus_stddev = []
    # noise_x_mean_minus_stddev = []
    # noise_y_mean = []
    # noise_y_stddev = []
    # noise_y_mean_plus_stddev = []
    # noise_y_mean_minus_stddev = []
    # noise_y_mean = []
    # noise_y_stddev = []
    # noise_y_mean_plus_stddev = []
    # noise_y_mean_minus_stddev = []
    # for i in range(0, len(pos_x[0])):
    #     pos_x_sum = 0
    #     pos_y_sum = 0
    #     pos_z_sum = 0
    #     noise_x_sum = 0
    #     noise_y_sum = 0
    #     noise_z_sum = 0
    #     for j in range(0, len(pos_x)):
    #         pos_x_sum += pos_x[j][i]
    #         pos_y_sum += pos_y[j][i]
    #         pos_z_sum += pos_z[j][i]
    #         noise_x_sum += noise_x[j][i]
    #         noise_y_sum += noise_y[j][i]
    #         noise_z_sum += noise_z[j][i]
    #     pos_x_mean.append(pos_x_sum / len(pos_x))
    #     pos_y_mean.append(pos_y_sum / len(pos_y))
    #     pos_z_mean.append(pos_z_sum / len(pos_z))
    #     noise_x_mean.append(noise_x_sum / len(noise_x))
    #     noise_y_mean.append(noise_y_sum / len(noise_y))
    #     noise_z_mean.append(noise_z_sum / len(noise_z))
    #     pos_x_stddev_sum = 0
    #     pos_y_stddev_sum = 0
    #     pos_z_stddev_sum = 0
    #     noise_x_stddev_sum = 0
    #     noise_y_stddev_sum = 0
    #     noise_z_stddev_sum = 0
    #     for j in range(0, len(pos_x)):
    #         pos_x_stddev_sum += (pos_x[j][i] - pos_x_mean[len(pos_x_mean) - 1]) ** 2
    #         pos_y_stddev_sum += (pos_y[j][i] - pos_y_mean[len(pos_y_mean) - 1]) ** 2
    #         pos_z_stddev_sum += (pos_z[j][i] - pos_z_mean[len(pos_z_mean) - 1]) ** 2
    #         noise_x_stddev_sum += (noise_x[j][i] - noise_x_mean[len(noise_x_mean) - 1]) ** 2
    #         noise_y_stddev_sum += (noise_y[j][i] - noise_y_mean[len(noise_y_mean) - 1]) ** 2
    #         noise_z_stddev_sum += (noise_z[j][i] - noise_z_mean[len(noise_z_mean) - 1]) ** 2
    #     pos_x_stddev.append(math.sqrt(pos_x_stddev_sum / len(pos_x)))
    #     pos_y_stddev.append(math.sqrt(pos_y_stddev_sum / len(pos_y)))
    #     pos_z_stddev.append(math.sqrt(pos_z_stddev_sum / len(pos_z)))
    #     noise_x_stddev.append(math.sqrt(noise_x_stddev_sum / len(noise_x)))
    #     noise_y_stddev.append(math.sqrt(noise_y_stddev_sum / len(noise_y)))
    #     noise_z_stddev.append(math.sqrt(noise_z_stddev_sum / len(noise_z)))
    #     pos_x_mean_plus_stddev.append(pos_x_mean[len(pos_x_mean) - 1] + pos_x_stddev[len(pos_x_stddev) - 1])
    #     pos_y_mean_plus_stddev.append(pos_y_mean[len(pos_y_mean) - 1] + pos_y_stddev[len(pos_y_stddev) - 1])
    #     pos_z_mean_plus_stddev.append(pos_z_mean[len(pos_z_mean) - 1] + pos_z_stddev[len(pos_z_stddev) - 1])
    #     noise_x_mean_plus_stddev.append(noise_x_mean[len(noise_x_mean) - 1] + noise_x_stddev[len(noise_x_stddev) - 1])
    #     noise_y_mean_plus_stddev.append(noise_y_mean[len(noise_y_mean) - 1] + noise_x_stddev[len(noise_y_stddev) - 1])
    #     noise_z_mean_plus_stddev.append(noise_z_mean[len(noise_z_mean) - 1] + noise_x_stddev[len(noise_z_stddev) - 1])
    #     pos_x_mean_minus_stddev.append(pos_x_mean[len(pos_x_mean) - 1] - pos_x_stddev[len(pos_x_stddev) - 1])
    #     pos_y_mean_minus_stddev.append(pos_y_mean[len(pos_y_mean) - 1] - pos_x_stddev[len(pos_y_stddev) - 1])
    #     pos_z_mean_minus_stddev.append(pos_z_mean[len(pos_z_mean) - 1] - pos_x_stddev[len(pos_z_stddev) - 1])
    #     noise_x_mean_minus_stddev.append(noise_x_mean[len(noise_x_mean) - 1] - noise_x_stddev[len(noise_x_stddev) - 1])
    #     noise_y_mean_minus_stddev.append(noise_y_mean[len(noise_x_mean) - 1] - noise_y_stddev[len(noise_y_stddev) - 1])
    #     noise_z_mean_minus_stddev.append(noise_z_mean[len(noise_x_mean) - 1] - noise_z_stddev[len(noise_z_stddev) - 1])
