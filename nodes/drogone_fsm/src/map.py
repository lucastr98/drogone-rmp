#!/usr/bin/env python
#2D Map for the GUI of DroGone
#Author: Felix Stadler, 1.5.20
import rospy
import roslib
import numpy as np
import tf
from tkinter import *
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from matplotlib import style, gridspec

from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory
from collections import deque #erases last elements if path arrays become to long

class Maps(object):

    def __init__(self, window): #window is Tk()

        self.maxLengthPath = 100 #duration*100 #length of path arrays -> number of points
        self.maxHeightPath = 100 #duration*100 #max length of z position(height) Path

        #  variables to store position of peregrine and the msg header time
        self.peregrine_path_x = deque([], maxlen=self.maxLengthPath)
        self.peregrine_path_y = deque([], maxlen=self.maxLengthPath)
        self.peregrine_path_z = deque([], maxlen=self.maxHeightPath)
        self.time_peregrine = deque([], maxlen=self.maxHeightPath)

        #  variables to store position of victim drone and the msg header time
        self.victim_path_x = deque([], maxlen=self.maxLengthPath)
        self.victim_path_y = deque([], maxlen=self.maxLengthPath)
        self.victim_path_z = deque([], maxlen=self.maxHeightPath)
        self.time_victim = deque([], maxlen=self.maxHeightPath)

        #future position of victim drone and the msg header time
        self.tracker_path_x = []
        self.tracker_path_y = []
        self.tracker_path_z = []
        self.time_tracker = []

        #position where trajectory intersects with victim drone path
        self.intersection_x = 0
        self.intersection_y = 0
        self.intersection_z = 0
        self.time_intersection = 0
        self.intersection_point = 0

        #planned trajectory of peregrine and the msg header time
        self.trajectory_x = []
        self.trajectory_y = []
        self.trajectory_z = []
        self.time_trajectory = []

        #help variables
        self.count_msgs = 0


        ## TODO: Transforms? might be useful:
        # self.listener = tf.TransformListener()

        #matplotlib
        style.use('ggplot')
        self.figure = Figure(figsize=(18,7), dpi = 100)
        gs = gridspec.GridSpec(1, 2, width_ratios=[3, 1])
        # xy position
        self.plot_xy = self.figure.add_subplot(gs[0])
        self.plot_xy.set_xlabel('x [m]')
        self.plot_xy.set_ylabel('y [m]')
        # TODO: set limit dynamic? for now same as rviz
        self.plot_xy.set_xlim(-38, 38)
        self.plot_xy.set_ylim(-23, 23)

        #z height over time
        self.plot_z = self.figure.add_subplot(gs[1])
        self.plot_z.set_xlabel('time [s]')
        self.plot_z.set_ylabel('z [m]')
        #only plot over the last 10 seconds + 2 seconds future tracker
        self.plot_z.set_xlim(0, 12)
        self.plot_z.set_ylim(0, 43)

        #xy position lines for peregrine, victim drone, tracker, intersection and trajectory
        self.xy_line_peregrine, self.xy_line_victim, self.xy_line_tracker, self.xy_line_intersection, self.xy_line_trajectory, self.xy_line_victim_curr, self.xy_line_peregrine_curr \
        = self.plot_xy.plot([], [], 'r-', [], [], 'g-', [], [], 'b.', [], [], 'm*', [], [], 'y-', [], [], 'go', [], [], 'ro')
        self.xy_line_peregrine.set_linewidth(1.5)
        self.xy_line_victim.set_linewidth(1.5)
        self.xy_line_trajectory.set_linewidth(1.5)
        self.xy_line_tracker.set_markersize(1)
        self.xy_line_intersection.set_markersize(4)
        self.xy_line_peregrine_curr.set_markersize(3)
        self.xy_line_victim_curr.set_markersize(3)

        #z height over time lines for peregrine, victim drone, tracker, intersection and trajectory
        self.z_line_peregrine, self.z_line_victim, self.z_line_tracker, self.z_line_intersection, self.z_line_trajectory \
        = self.plot_z.plot([], [], 'r-', [], [], 'g-', [], [], 'b.', [], [], 'm*', [], [], 'y-')
        self.z_line_peregrine.set_linewidth(1.5)
        self.z_line_victim.set_linewidth(1.5)
        self.z_line_trajectory.set_linewidth(1.5)
        self.z_line_tracker.set_markersize(1)
        self.z_line_intersection.set_markersize(4)

        #add plot to tkinter window
        self.canvas = FigureCanvasTkAgg(self.figure, master=window)
        self.canvas.draw()
        toolbar_frame = Frame(window)
        toolbar_frame.grid(row = 9, columnspan = 3)
        self.toolbar = NavigationToolbar2TkAgg(self.canvas, toolbar_frame)
        self.toolbar.update()
        self.canvas.get_tk_widget().grid(rowspan=8, columnspan=8)

        self.main_label = Label(window, text='Map of xy Position', font="Sans 12 bold")
        self.main_label.grid(row=1, column=0)
        self.main_label = Label(window, text='Map of z Position', font="Sans 12 bold")
        self.main_label.grid(row=1, column=2)

        self.pause = 0 #pause the plotting of new positions of the drones and plots planned trajectory of peregrine -> debug tool for motion planning
        self.Pause_Plot = StringVar()
        self.Pause_Plot.set("Pause plotting")
        Button(window, textvariable=self.Pause_Plot, command=self.pause_plotting).grid(row=1, column=1)

        #intersection point
        Label(window, textvariable=self.intersection_point, height = 2, width = 5).grid(row=1,column=4)


    def peregrine_odom_cb(self, data):
        #UPDATE with 10 hz
        if self.pause == 1 or self.count_msgs < 9:
            self.count_msgs += 1
        else:
            # store position and time in path
            self.peregrine_path_x.append(data.pose.pose.position.x)
            self.peregrine_path_y.append(data.pose.pose.position.y)
            self.xy_line_peregrine.set_xdata(self.peregrine_path_x)
            self.xy_line_peregrine.set_ydata(self.peregrine_path_y)
            #show current xy position
            self.xy_line_peregrine_curr.set_xdata(data.pose.pose.position.x)
            self.xy_line_peregrine_curr.set_ydata(data.pose.pose.position.y)

            self.peregrine_path_z.append(data.pose.pose.position.z)
            seconds = float(data.header.stamp.secs) + float(data.header.stamp.nsecs) / float(1e9)
            self.time_peregrine.append(seconds)
            self.z_line_peregrine.set_xdata(self.time_peregrine)
            self.z_line_peregrine.set_ydata(self.peregrine_path_z)
            self.plot_z.set_xlim(min(self.time_peregrine),
                                 max(list(self.time_peregrine) + list(self.time_tracker)) + 1)
            # if not self.pause:
            self.canvas.draw_idle()
            self.count_msgs = 0

    def tracker_cb(self, data):
        if not self.pause == 1:
            # store current position and time in path
            self.victim_path_x.append(data.points[0].transforms[0].translation.x)
            self.victim_path_y.append(data.points[0].transforms[0].translation.y)
            self.xy_line_victim.set_xdata(self.victim_path_x)
            self.xy_line_victim.set_ydata(self.victim_path_y)
            #show current position
            self.xy_line_victim_curr.set_xdata(data.points[0].transforms[0].translation.x)
            self.xy_line_victim_curr.set_ydata(data.points[0].transforms[0].translation.y)

            self.victim_path_z.append(data.points[0].transforms[0].translation.z)
            seconds = float(data.header.stamp.secs) + float(data.header.stamp.nsecs) / float(1e9)
            self.time_victim.append(seconds)
            self.z_line_victim.set_xdata(self.time_victim)
            self.z_line_victim.set_ydata(self.victim_path_z)

            #future positions
            future_path_x = []
            future_path_y = []
            future_path_z = []
            future_time = []
            for i in range (1,len(data.points)): #start with [1] as [0] is current position
                future_path_x.append(data.points[i].transforms[0].translation.x)
                future_path_y.append(data.points[i].transforms[0].translation.y)
                future_path_z.append(data.points[i].transforms[0].translation.z)
                seconds = float(data.points[i].time_from_start.secs) + float(data.points[i].time_from_start.nsecs) / float(1e9)
                future_time.append(seconds)
            self.tracker_path_x = future_path_x
            self.tracker_path_y = future_path_y
            self.xy_line_tracker.set_xdata(self.tracker_path_x)
            self.xy_line_tracker.set_ydata(self.tracker_path_y)
            self.tracker_path_z = future_path_z
            self.time_tracker = future_time
            self.z_line_tracker.set_xdata(self.time_tracker)
            self.z_line_tracker.set_ydata(self.tracker_path_z)

    def intersection_cb(self, data):
        if not self.pause == 1 and len(self.tracker_path_x) > 0 :
            self.intersection_point = data.selected_keys[1] - 2 #-1 because selected keys array starts with drogone position and another -1  because self.tracker_path_x does not contain the first element of the tracker msg (the victim drone position)
            self.intersection_x = self.tracker_path_x[self.intersection_point]
            self.intersection_y = self.tracker_path_y[self.intersection_point]
            self.xy_line_intersection.set_xdata(self.intersection_x)
            self.xy_line_intersection.set_ydata(self.intersection_y)
            self.intersection_z = self.tracker_path_z[self.intersection_point]
            self.time_intersection = self.time_tracker[self.intersection_point]
            self.z_line_intersection.set_xdata(self.time_intersection)
            self.z_line_intersection.set_ydata(self.intersection_z)

    def trajectory_cb(self, data):
        # only plot if plotting paused
        if self.pause == 2:
            future_path_x = []
            future_path_y = []
            future_path_z = []
            future_time = []
            for i in range (0,len(data.points),10):
                future_path_x.append(data.points[i].transforms[0].translation.x)
                future_path_y.append(data.points[i].transforms[0].translation.y)
                future_path_z.append(data.points[i].transforms[0].translation.z)
                seconds = float(data.points[i].time_from_start.secs) + float(data.points[i].time_from_start.nsecs) / float(1e9)
                future_time.append(seconds)
            self.trajectory_x = future_path_x
            self.trajectory_y = future_path_y
            self.xy_line_trajectory.set_xdata(self.trajectory_x)
            self.xy_line_trajectory.set_ydata(self.trajectory_y)
            self.trajectory_z = future_path_z
            self.time_trajectory = future_time
            self.z_line_trajectory.set_xdata(self.time_trajectory)
            self.z_line_trajectory.set_ydata(self.trajectory_z)
            self.plot_z.set_xlim(min(self.time_peregrine),
                                 max(list(self.time_peregrine) + list(self.time_tracker)+ list(self.time_trajectory)) + 1)
            self.canvas.draw_idle()
            self.pause = 1

    def pause_plotting(self):
        if self.Pause_Plot.get() == "Pause plotting":
            self.pause = 2
            current_time = rospy.Time.now()
            while (rospy.get_rostime() - current_time  < rospy.Duration(0.2)):
                continue
            self.pause = 1
            self.Pause_Plot.set("Continue plotting")

        elif self.Pause_Plot.get() == "Continue plotting":
            self.pause = 0
            self.Pause_Plot.set("Pause plotting")
