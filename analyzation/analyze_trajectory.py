import rosbag
import numpy as np
import tf
import math
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt

################################################################################
############################## FOR USER TO CHANGE ##############################
################################################################################

# fill in correct rosbag
bag = rosbag.Bag('roll_pitch_active.bag')

# if true plot acceleration, if false plot roll/pitch
plot_acc = 1

################################################################################
################################################################################
################################################################################

fig, axes = plt.subplots(3, 3, sharex=True)
fig, axes2 = plt.subplots(1, 2, gridspec_kw={'width_ratios': [3, 1]})

# odometry of the drone
odometry_pos_x = []
odometry_pos_y = []
odometry_pos_z = []
odometry_lin_vel_x = []
odometry_lin_vel_y = []
odometry_lin_vel_z = []
odometry_roll = []
odometry_pitch = []
odometry_time = []
for topic, msg, t in bag.read_messages(topics='/peregrine/odometry_sensor1/odometry'):
    odometry_pos_x.append(msg.pose.pose.position.x)
    odometry_pos_y.append(msg.pose.pose.position.y)
    odometry_pos_z.append(msg.pose.pose.position.z)

    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w

    t0 = +2.0 * (qw * qx + qy * qz)
    t1 = +1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (qw * qy - qz * qx)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    odometry_roll.append(roll)
    odometry_pitch.append(pitch)

    mat11 = 1 - 2*qy*qy - 2*qz*qz
    mat12 = 2*qx*qy - 2*qz*qw
    mat13 = 2*qx*qz + 2*qy*qw
    mat21 = 2*qx*qy + 2*qz*qw
    mat22 = 1 - 2*qx*qx - 2*qz*qz
    mat23 = 2*qy*qz - 2*qx*qw
    mat31 = 2*qx*qz - 2*qy*qw
    mat32 = 2*qy*qz + 2*qx*qw
    mat33 = 1 - 2*qx*qx - 2*qy*qy
    v_x_body = msg.twist.twist.linear.x
    v_y_body = msg.twist.twist.linear.y
    v_z_body = msg.twist.twist.linear.z
    odometry_lin_vel_x.append(mat11*v_x_body + mat12*v_y_body + mat13*v_z_body)
    odometry_lin_vel_y.append(mat21*v_x_body + mat22*v_y_body + mat23*v_z_body)
    odometry_lin_vel_z.append(mat31*v_x_body + mat32*v_y_body + mat33*v_z_body)
    odometry_time.append(msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)

# trajectory sent to drone
trajectory_pos_x_list = []
trajectory_pos_y_list = []
trajectory_pos_z_list = []
trajectory_lin_vel_x_list = []
trajectory_lin_vel_y_list = []
trajectory_lin_vel_z_list = []
trajectory_lin_acc_x_list = []
trajectory_lin_acc_y_list = []
trajectory_lin_acc_z_list = []
trajectory_time_list = []
for topic, msg, t in bag.read_messages(topics='/peregrine/command/trajectory'):
    number_of_points = len(msg.points)
    trajectory_pos_x = []
    trajectory_pos_y = []
    trajectory_pos_z = []
    trajectory_lin_vel_x = []
    trajectory_lin_vel_y = []
    trajectory_lin_vel_z = []
    trajectory_lin_acc_x = []
    trajectory_lin_acc_y = []
    trajectory_lin_acc_z = []
    trajectory_time = []
    for i in range(number_of_points):
        trajectory_pos_x.append(msg.points[i].transforms[0].translation.x)
        trajectory_pos_y.append(msg.points[i].transforms[0].translation.y)
        trajectory_pos_z.append(msg.points[i].transforms[0].translation.z)
        trajectory_lin_vel_x.append(msg.points[i].velocities[0].linear.x)
        trajectory_lin_vel_y.append(msg.points[i].velocities[0].linear.y)
        trajectory_lin_vel_z.append(msg.points[i].velocities[0].linear.z)
        trajectory_lin_acc_x.append(msg.points[i].accelerations[0].linear.x)
        trajectory_lin_acc_y.append(msg.points[i].accelerations[0].linear.y)
        trajectory_lin_acc_z.append(msg.points[i].accelerations[0].linear.z)
        trajectory_time.append(msg.points[i].time_from_start.secs +
                               msg.points[i].time_from_start.nsecs/1e9 +
                               msg.header.stamp.secs +
                               msg.header.stamp.nsecs/1e9)
    trajectory_pos_x_list.append(trajectory_pos_x)
    trajectory_pos_y_list.append(trajectory_pos_y)
    trajectory_pos_z_list.append(trajectory_pos_z)
    trajectory_lin_vel_x_list.append(trajectory_lin_vel_x)
    trajectory_lin_vel_y_list.append(trajectory_lin_vel_y)
    trajectory_lin_vel_z_list.append(trajectory_lin_vel_z)
    trajectory_lin_acc_x_list.append(trajectory_lin_acc_x)
    trajectory_lin_acc_y_list.append(trajectory_lin_acc_y)
    trajectory_lin_acc_z_list.append(trajectory_lin_acc_z)
    trajectory_time_list.append(trajectory_time)

# command sent to drone
command_roll = []
command_pitch = []
command_time = []
for topic, msg, t in bag.read_messages(topics='/peregrine/command/roll_pitch_yawrate_thrust'):
    command_roll.append(msg.roll)
    command_pitch.append(msg.pitch)
    command_time.append(msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)

# IMU acceleration
acceleration_x = []
acceleration_y = []
acceleration_z = []
acceleration_time = []
for topic, msg, t in bag.read_messages(topics= '/peregrine/imu'):
    x = msg.orientation.x
    y = msg.orientation.y
    z = msg.orientation.z
    w = msg.orientation.w
    mat11 = 1 - 2*y*y - 2*z*z
    mat12 = 2*x*y - 2*z*w
    mat13 = 2*x*z + 2*y*w
    mat21 = 2*x*y + 2*z*w
    mat22 = 1 - 2*x*x - 2*z*z
    mat23 = 2*y*z - 2*x*w
    mat31 = 2*x*z - 2*y*w
    mat32 = 2*y*z + 2*x*w
    mat33 = 1 - 2*x*x - 2*y*y
    a_x_body = msg.linear_acceleration.x
    a_y_body = msg.linear_acceleration.y
    a_z_body = msg.linear_acceleration.z
    acceleration_x.append(mat11*a_x_body + mat12*a_y_body + mat13*a_z_body)
    acceleration_y.append(mat21*a_x_body + mat22*a_y_body + mat23*a_z_body)
    acceleration_z.append(mat31*a_x_body + mat32*a_y_body + mat33*a_z_body - 9.81)
    acceleration_time.append(msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)

# plot what was flown
axes[0][0].plot(odometry_time, odometry_pos_x, '.', label='odometry', c='b')
axes[1][0].plot(odometry_time, odometry_pos_y, '.', label='odometry', c='b')
axes[2][0].plot(odometry_time, odometry_pos_z, '.', label='odometry', c='b')
axes[0][1].plot(odometry_time, odometry_lin_vel_x, '.', label='odometry', c='b')
axes[1][1].plot(odometry_time, odometry_lin_vel_y, '.', label='odometry', c='b')
axes[2][1].plot(odometry_time, odometry_lin_vel_z, '.', label='odometry', c='b')
if(plot_acc == 0):
    axes[0][2].plot(odometry_time, odometry_roll, '.', label='odometry', c='b')
    axes[1][2].plot(odometry_time, odometry_pitch, '.', label='odometry', c='b')
else:
    axes[0][2].plot(acceleration_time, acceleration_x, '.', label='IMU', c='b')
    axes[1][2].plot(acceleration_time, acceleration_y, '.', label='IMU', c='b')
    axes[2][2].plot(acceleration_time, acceleration_z, '.', label='IMU', c='b')

# plot what was commanded
number_of_traj = len(trajectory_pos_x_list)
for i in range(0, number_of_traj, 1):
    axes[0][0].plot(trajectory_time_list[i], trajectory_pos_x_list[i], '.', label='trajectory', c='r')
    axes[1][0].plot(trajectory_time_list[i], trajectory_pos_y_list[i], '.', label='trajectory', c='r')
    axes[2][0].plot(trajectory_time_list[i], trajectory_pos_z_list[i], '.', label='trajectory', c='r')
    axes[0][1].plot(trajectory_time_list[i], trajectory_lin_vel_x_list[i], '.', label='trajectory', c='r')
    axes[1][1].plot(trajectory_time_list[i], trajectory_lin_vel_y_list[i], '.', label='trajectory', c='r')
    axes[2][1].plot(trajectory_time_list[i], trajectory_lin_vel_z_list[i], '.', label='trajectory', c='r')
    if(plot_acc == 1):
        axes[0][2].plot(trajectory_time_list[i], trajectory_lin_acc_x_list[i], '.', label='trajectory', c='r')
        axes[1][2].plot(trajectory_time_list[i], trajectory_lin_acc_y_list[i], '.', label='trajectory', c='r')
        axes[2][2].plot(trajectory_time_list[i], trajectory_lin_acc_z_list[i], '.', label='trajectory', c='r')
if(plot_acc == 0):
    axes[0][2].plot(command_time, command_roll, '.', label='command', c='r')
    axes[1][2].plot(command_time, command_pitch, '.', label='command', c='r')

axes[0][0].grid()
axes[1][0].grid()
axes[2][0].grid()
axes[0][1].grid()
axes[1][1].grid()
axes[2][1].grid()
axes[0][2].grid()
axes[1][2].grid()
axes[2][2].grid()

axes[0][0].set(title='position in x', xlabel='time [s]', ylabel='position [m]')
axes[1][0].set(title='position in y', xlabel='time [s]', ylabel='position [m]')
axes[2][0].set(title='position in z', xlabel='time [s]', ylabel='position [m]')
axes[0][1].set(title= 'linear velocity in x', xlabel='time [s]', ylabel='velocity [m/s]')
axes[1][1].set(title= 'linear velocity in y', xlabel='time [s]', ylabel='velocity [m/s]')
axes[2][1].set(title= 'linear velocity in z', xlabel='time [s]', ylabel='velocity [m/s]')
if(plot_acc == 1):
    axes[0][2].set(title= 'linear acceleration in x', xlabel='time [s]', ylabel='velocity [m/s/s]')
    axes[1][2].set(title= 'linear acceleration in y', xlabel='time [s]', ylabel='velocity [m/s/s]')
    axes[2][2].set(title= 'linear acceleration in z', xlabel='time [s]', ylabel='velocity [m/s/s]')
else:
    axes[0][2].set(title= 'roll', xlabel='time [s]', ylabel='angle [rad]')
    axes[1][2].set(title= 'pitch', xlabel='time [s]', ylabel='angle [rad]')

trajectory_handle = mpatches.Patch(color='r', label='trajectory')
command_handle = mpatches.Patch(color='r', label='command')
odometry_handle = mpatches.Patch(color='b', label='odometry')
IMU_handle = mpatches.Patch(color='b', label='IMU')
MPC_handle = mpatches.Patch(color='g', label='MPC')

axes[0][0].legend(handles=[odometry_handle, trajectory_handle])
axes[1][0].legend(handles=[odometry_handle, trajectory_handle])
axes[2][0].legend(handles=[odometry_handle, trajectory_handle])
axes[0][1].legend(handles=[odometry_handle, trajectory_handle])
axes[1][1].legend(handles=[odometry_handle, trajectory_handle])
axes[2][1].legend(handles=[odometry_handle, trajectory_handle])
if(plot_acc == 1):
    axes[0][2].legend(handles=[IMU_handle, trajectory_handle])
    axes[1][2].legend(handles=[IMU_handle, trajectory_handle])
    axes[2][2].legend(handles=[IMU_handle, trajectory_handle])
else:
    axes[0][2].legend(handles=[odometry_handle, command_handle])
    axes[1][2].legend(handles=[odometry_handle, command_handle])

f_u_list = []
f_v_list = []
u_list = []
v_list = []
t_list = []
counter = 0
counter2 = 0
for topic, msg, t in bag.read_messages(topics= '/f_u_v'):
    f_u_list.append(msg.f_u)
    f_v_list.append(msg.f_v)
    u_list.append(msg.x_u)
    v_list.append(msg.x_v)
    # v_list.append(msg.x_v - counter2)
    # counter2 += 10
    t_list.append(msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)

    # if(abs(msg.x_u) < 0.15):
    #     counter += 1
    # if(counter == 2):
    #     break

axes2[0].plot(u_list, v_list, '-')
axes2[1].plot(t_list, u_list, '-')

quiver_u_list = []
quiver_v_list = []
quiver_f_u_list = []
quiver_f_v_list = []
plotter = 5
for i in range(0, len(u_list) - 1, 1):
    if(i % plotter == 0):
        quiver_u_list.append(u_list[i])
        quiver_v_list.append(v_list[i])
        quiver_f_u_list.append(f_u_list[i])
        quiver_f_v_list.append(f_v_list[i])
axes2[0].quiver(quiver_u_list, quiver_v_list, quiver_f_u_list, quiver_f_v_list, units='xy') #'height')

axes2[0].set_xlim([-1024, 1024])
axes2[0].set_ylim([-768, 768])
axes2[0].set(title= 'camera image', xlabel='pixel', ylabel='pixel')
# axes2[0].set(title= 'camera image', xlabel='pixel', ylabel='v is constant, just varying v to see acc arrows')
axes2[1].set(title= 'u over time', xlabel='time', ylabel='pixel')
axes2[0].grid()
axes2[1].grid()

plt.show()
