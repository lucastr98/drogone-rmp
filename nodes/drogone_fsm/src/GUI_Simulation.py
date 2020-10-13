#!/usr/bin/env python
#GUI for DroGone Simulation
#Author: Felix Stadler, 26.11.19
from tkinter import *
from tkinter import ttk
import tkinter.messagebox as tkm
import rospy
import roslib
import actionlib
import time
import numpy as np
import drogone_action.msg #import Servermessage
from std_srvs.srv import Empty # back_to_position_hold call service
from std_msgs.msg import Bool, Int8, Float32MultiArray
from nav_msgs.msg import Odometry #for publishing point if detector is not working
from trajectory_msgs.msg import MultiDOFJointTrajectory
from drogone_msgs.msg import DijkstraViz
from map import Maps #import class map for GUI map

# from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg #change backend
# from matplotlib.figure import Figure
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
# from matplotlib.animation import FuncAnimation
# from matplotlib import style

class GUIServer(object):
    #create message that is used to publish result
    result_ = drogone_action.msg.FSMResult()
    goal_ = drogone_action.msg.FSMGoal()
    #help variable
    done_ = 0

#---------------------------------------------------ACTION SERVER for FSM--------------------------------------------------------------------------------------------------------------
    #define Constructor
    def __init__(self,name):
        self._action_name = name
        sub_1 = rospy.Subscriber("/GUI/communication", Int8, self.callback, queue_size=10)
        #subscribe to communication between FSM and MOtionplanner and Dummy Node
        check_for_catch_status = rospy.Subscriber("Check/result", drogone_action.msg.FSMActionResult, self.check_for_catch_status , queue_size=10)
        check_for_catch_status = rospy.Subscriber("Motionplanner/result", drogone_action.msg.FSMActionResult, self.motion_planner_status , queue_size=10)
        self._as = actionlib.SimpleActionServer(self._action_name, drogone_action.msg.FSMAction, execute_cb=self.execute_cb, auto_start = False )
        self._as.start()


    #define ActionServer execute_callback function
    def execute_cb(self, goal):

        current_state.set(goal.new_mode)

        if goal.new_mode == "StartManual":
            command_1.set("Take Off")
            command_2.set("no interaction possible")

        elif goal.new_mode == "WaitforAutonomous":
            command_1.set("Start Autonomous Flying")
            command_2.set("no interaction possible")

        elif goal.new_mode == "Follow":
            command_1.set("Catch Victim Drone")
            command_2.set("Victim Drone Lost")

        elif goal.new_mode == "Catch":
            command_1.set("Victim Drone Caught")
            # command_2.set("Victim Drone Lost")
            command_2.set("Restart")

        elif goal.new_mode == "WaitforInstruction":
            command_1.set("Land")
            command_2.set("Restart")

        else:
            command_1.set("no interaction possible")
            command_2.set("no interaction possible")


        while (not self._as.is_preempt_requested()) and (not rospy.is_shutdown()) and (not self.done_):
            continue

        if (self._as.is_preempt_requested() or rospy.is_shutdown()):
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted(self.result_)

        elif self.done_==1:
             rospy.loginfo('%s: Succeeded' % self._action_name)
             self._as.set_succeeded(self.result_)

        elif self.done_==2:
             rospy.loginfo('%s: Aborted' % self._action_name)
             self._as.set_aborted(self.result_)

        self.done_ = 0


    #define callback funciton of subscriber for GUI intern communication
    def callback(self, data):
        self.done_ = data.data

    #define callback function of check_for_catch status
    def check_for_catch_status(self, data):
        last_task.set("Check for Catch")
        if data.status.status == 3:
            task_status.set("Drone caught")
            parameters = Float32MultiArray()
            parameters.data = [0] * size_of_array
            parameters.data[12] = 1
            pub_victim.publish(parameters)
            #pause plotting
            rospy.sleep(1.5)
            maps.pause_plotting()
        elif data.status.status == 2:
            task_status.set("preempted")

    #define callback function of motionplanner status
    def motion_planner_status(self, data):
        if not current_state.get() == "Catch":
            last_task.set(current_state.get())
            if data.status.status == 2:
                task_status.set("preempted")
            elif data.status.status == 3:
                    task_status.set("succeeded")
            elif data.status.status == 4:
                task_status.set("aborted")


#callback function of button command
def command_1_callback():

    if command_1.get() == "Take Off" or command_1.get() == "Catch Victim Drone"  or command_1.get() == "Land":
        job_done = 1
        pub.publish(job_done)
        last_task.set("Autonomous Flying")
        task_status.set("on")

    elif command_1.get() == "Start Autonomous Flying":
         job_done = 1
         pub.publish(job_done)
         last_task.set("Autonomous Flying")
         task_status.set("on")

    elif command_1.get() == "Victim Drone Caught":
        last_task.set("GUI")
        task_status.set("Drone Caught")
        parameters = Float32MultiArray()
        parameters.data = [0] * size_of_array
        parameters.data[12] = 1
        pub_victim.publish(parameters)
        job_done = 1
        pub.publish(job_done)



def command_2_callback():
    if command_2.get() == "Victim Drone Lost":
        job_done = 2
        pub.publish(job_done)

    if command_2.get() == "Restart":
        last_task.set("Restart")
        task_status.set("-")
        parameters = Float32MultiArray()
        parameters.data = [0] * size_of_array
        parameters.data[10] = 1
        pub_victim.publish(parameters)
        #continue plotting
        maps.pause_plotting()
        job_done = 2
        pub.publish(job_done)


# ---------------------------------------------------VICTIM DRONE parameters --------------------------------------------------------------------------------------------------------------
# parameters are stored in an array with 18 entires (size_of_array):
# [0] = type of movement: hovering = 0; linear path = 1; circle = 2; eight = 3; random = 4;
# [1] = velocity
# [2] = change starting point, true/false
# [3] = new x starting point
# [4] = new y starting point
# [5] = new z starting point
# [6] = change motion vector for linear moving path, true/false
# [7] = new x component motion vector for linear moving path
# [8] = new y component motion vector for linear moving path
# [9] = new z component motion vector for linear moving path
# [10] = restart -> move victim drone back to starting position and hover
# [11] empty for now
# [12] = drone caught, true/flase
# [13] = new radius of circle/eight
# [14] = mew theta angle tilt in radian
# [15] = new distance between circles for eight
# [16] = theta angle width for random path
# [17] = phi angle width for random path


#linear moving drone
def linear_path_callback():
    parameters = Float32MultiArray()
    parameters.data = [0] * size_of_array
    if linear_path.get() == "linear path":
        parameters.data[0] = 1
        #take entry values
        if not len(victim_drone_velocity.get()) == 0:
            parameters.data[1] = float(victim_drone_velocity.get())
        if not len(victim_drone_motion_vector_x.get()) == 0:
            parameters.data[7] = float(victim_drone_motion_vector_x.get())
            parameters.data[6] = 1
        if not len(victim_drone_motion_vector_y.get()) == 0:
            parameters.data[8] = float(victim_drone_motion_vector_y.get())
            parameters.data[6] = 1
        if not len(victim_drone_motion_vector_z.get()) == 0:
            parameters.data[9] = float(victim_drone_motion_vector_z.get())
            parameters.data[6] = 1

        # for evaluation s.t. the random direction is always used
        parameters.data[6] = 1

        pub_victim.publish(parameters)
        command_1_callback()

#circular moving drone
def circular_path_callback():
    parameters = Float32MultiArray()
    parameters.data = [0] * size_of_array
    if circular_path.get() == "circular path":
        parameters.data[0] = 2
        #take entry values
        if not len(victim_drone_velocity.get()) == 0:
            parameters.data[1] = float(victim_drone_velocity.get())
        if not len(victim_drone_radius.get()) == 0:
            parameters.data[13] = float(victim_drone_radius.get())
        if not len(victim_drone_theta.get()) == 0:
            parameters.data[14] = float(victim_drone_theta.get())
        pub_victim.publish(parameters)
        rospy.sleep(0.4)
        command_1_callback()

#eight path moving drone
def eight_path_callback():
    parameters = Float32MultiArray()
    parameters.data = [0] * size_of_array
    if eight_path.get() == "eight path":
        parameters.data[0] = 3
        #take entry values
        if not len(victim_drone_velocity.get()) == 0:
            parameters.data[1] = float(victim_drone_velocity.get())
        if not len(victim_drone_radius.get()) == 0:
            parameters.data[13] = float(victim_drone_radius.get())
        if not len(victim_drone_theta.get()) == 0:
            parameters.data[14] = float(victim_drone_theta.get())
        if not len(victim_drone_distance.get()) == 0:
            parameters.data[15] = float(victim_drone_distance.get())
        pub_victim.publish(parameters)
        command_1_callback()

#random moving drone
def random_path_callback():
    parameters = Float32MultiArray()
    parameters.data = [0] * size_of_array
    if random_path.get() == "random path":
        parameters.data[0] = 4
        #take entry values
        if not len(victim_drone_velocity.get()) == 0:
            parameters.data[1] = float(victim_drone_velocity.get())
        if not len(victim_drone_theta_width.get()) == 0:
            parameters.data[16] = float(victim_drone_theta_width.get())
        if not len(victim_drone_phi_width.get()) == 0:
            parameters.data[17] = float(victim_drone_phi_width.get())

        # for evaluation s.t. the random direction is always used
        parameters.data[6] = 1

        pub_victim.publish(parameters)
        command_1_callback()

#stop drone
def stop_path_callback():
    parameters = Float32MultiArray()
    parameters.data = [0] * size_of_array
    parameters.data[0] = 0
    pub_victim.publish(parameters)

#change starting point
def change_starting_point_callback():
    parameters = Float32MultiArray()
    parameters.data = [0] * size_of_array
    if change_starting_point.get() == "Change starting point (x,y,z):":
        parameters.data[2] = 1
        #take entry values
        if not len(starting_point_x.get()) == 0:
            parameters.data[3] = float(starting_point_x.get())
        if not len(starting_point_y.get()) == 0:
            parameters.data[4] = float(starting_point_y.get())
        if not len(starting_point_z.get()) == 0:
            parameters.data[5] = float(starting_point_z.get())

        if len(starting_point_x.get()) == 0 and len(starting_point_y.get()) == 0 and len(starting_point_z.get()) == 0:
            parameters.data[18] = 1
        else:
            parameters.data[18] = 0

        pub_victim.publish(parameters)

if __name__ == '__main__':
    rospy.init_node('DroGone_GUI')
    rospy.logwarn(rospy.get_name() + ' IS RUNNING')

    # initialize window
    window = Tk()
    window.title("GUI DroGone")
    #define size/geometry
    window.geometry('1900x1000')

    #frame surrounding other widgets
    frame = Frame(window)
    frame.grid()

    maps = Maps(window)
    #Publishers
    pub = rospy.Publisher("/GUI/communication", Int8, queue_size=10)
    pub_victim = rospy.Publisher("/GUI/victim_drone_param", Float32MultiArray, queue_size=10)

    #initilaize Subscribers
    rospy.Subscriber("odometry_sensor1/odometry",  Odometry, maps.peregrine_odom_cb, queue_size=10)
    rospy.Subscriber("ground_truth_tracker",  MultiDOFJointTrajectory, maps.tracker_cb, queue_size=10)
    rospy.Subscriber("command/trajectory",  MultiDOFJointTrajectory, maps.trajectory_cb, queue_size=100)
    rospy.Subscriber("/visualization/Dijkstra", DijkstraViz, maps.intersection_cb, queue_size=100)

    # size of array for communication iwith victim drone
    size_of_array = 19




    #----------------------------------------------------------------------interaction with FSM--------------------------------------------------------
    #current state
    Label(frame, text="Current State:", height = 2, width = 20).grid(row=0,column=0,sticky=W)
    # information about the states from the FSM
    current_state = StringVar()
    current_state.set("GUI is starting...")
    Label(frame, textvariable=current_state, height = 2, width = 20).grid(row=1,column=0,sticky=W)
    last_task = StringVar()
    last_task.set("")
    # Label(frame, textvariable=last_task, height = 2, width = 20).grid(row=3,column=0,sticky=W)
    task_status = StringVar()
    current_state.set("")
    Label(frame, textvariable=task_status, height = 2, width = 20).grid(row=3,column=1,sticky=W)

    #Interaction with FSM
    Label(frame, text="Interaction:", height = 2, width = 20).grid(row=0,column=1,sticky=W)
    command_1 = StringVar()
    command_1.set("Waiting for interaction")
    Button(frame, textvariable=command_1, command=command_1_callback).grid(row=1, column=1)
    command_2 = StringVar()
    command_2.set("Waiting for interaction")
    Button(frame, textvariable=command_2, command=command_2_callback).grid(row=2, column=1)


    #----------------------------------------------------------------------path of victim drone parameters--------------------------------------------------------
    #Start moving of Victim Drone
    Label(frame, text="Victim Drone:", height = 2, width = 20).grid(row=0,column=2,sticky=W)

    #moving paths
    linear_path = StringVar()
    linear_path.set("linear path")
    Button(frame, textvariable=linear_path, command=linear_path_callback).grid(row=1, column=2)
    circular_path = StringVar()
    circular_path.set("circular path")
    Button(frame, textvariable=circular_path, command=circular_path_callback).grid(row=2, column=2)
    eight_path = StringVar()
    eight_path.set("eight path")
    Button(frame, textvariable=eight_path, command=eight_path_callback).grid(row=3, column=2)
    random_path = StringVar()
    random_path.set("random path")
    Button(frame, textvariable=random_path, command=random_path_callback).grid(row=4, column=2)
    stop_path = StringVar()
    stop_path.set("stop Victim Drone")
    Button(frame, textvariable=stop_path, command=stop_path_callback).grid(row=6, column=2)


    Label(frame, text="Victim Drone parameters:", height = 2, width = 25).grid(row=0, column=3, sticky=W)
    #motion vector for linear moving drone
    Label(frame, text="velocity vector (x,y,z):", height = 2, width = 20).grid(row=1, column=3,sticky=W)
    victim_drone_motion_vector_x = StringVar()
    victim_drone_motion_vector_x.set("")
    Entry(frame, textvariable = victim_drone_motion_vector_x,  width = 5).grid(row=1, column=4)
    victim_drone_motion_vector_y = StringVar()
    victim_drone_motion_vector_y.set("")
    Entry(frame, textvariable = victim_drone_motion_vector_y,  width = 5).grid(row=1, column=5)
    victim_drone_motion_vector_z = StringVar()
    victim_drone_motion_vector_z.set("")
    Entry(frame, textvariable = victim_drone_motion_vector_z,  width = 5).grid(row=1, column=6)

    #radius entry
    Label(frame, text="radius & tilt (degree):", height = 2, width = 20).grid(row=2,column=3,sticky=W)
    victim_drone_radius = StringVar()
    victim_drone_radius.set("")
    Entry(frame, textvariable = victim_drone_radius,  width = 5).grid(row=2, column=4)
    victim_drone_theta = StringVar()
    victim_drone_theta.set("")
    Entry(frame, textvariable = victim_drone_theta,  width = 5).grid(row=2, column=6)

    #distance entry
    Label(frame, text="distance of half circles:", height = 2, width = 20).grid(row=3,column=3,sticky=W)
    victim_drone_distance = StringVar()
    victim_drone_distance.set("")
    Entry(frame, textvariable = victim_drone_distance,  width = 5).grid(row=3, column=4)

    #velocity entry
    Label(frame, text="Drone velocity:", height = 2, width = 20).grid(row=5,column=3,sticky=W)
    victim_drone_velocity = StringVar()
    victim_drone_velocity.set("")
    Entry(frame, textvariable = victim_drone_velocity,  width = 5).grid(row=5, column=4)

    #angle width entry
    Label(frame, text="theta & phi width:", height = 2, width = 20).grid(row=4,column=3,sticky=W)
    victim_drone_theta_width = StringVar()
    victim_drone_theta_width.set("")
    Entry(frame, textvariable = victim_drone_theta_width,  width = 5).grid(row=4, column=4)
    victim_drone_phi_width = StringVar()
    victim_drone_phi_width.set("")
    Entry(frame, textvariable = victim_drone_phi_width,  width = 5).grid(row=4, column=6)
    victim_drone_phi_width

    #starting point
    change_starting_point = StringVar()
    change_starting_point.set("Change starting point (x,y,z):")
    Button(frame, textvariable=change_starting_point, command=change_starting_point_callback).grid(row=6, column=3)
    starting_point_x = StringVar()
    starting_point_x.set("")
    Entry(frame, textvariable = starting_point_x, width = 5).grid(row=6, column=4)
    starting_point_y = StringVar()
    starting_point_y.set("")
    Entry(frame, textvariable = starting_point_y,  width = 5).grid(row=6, column=5)
    starting_point_z = StringVar()
    starting_point_z.set("")
    Entry(frame, textvariable = starting_point_z,  width = 5).grid(row=6, column=6)

    #initialize GUI Server class
    GUIServer = GUIServer("GUI")

    #display window till manually closed
    window.mainloop()

    rospy.spin() #rospyspin keeps node running
