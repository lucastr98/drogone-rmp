#!/usr/bin/env python
#GUI for MBZIRC
#Author: Felix Stadler, 26.11.19
from tkinter import *
import tkinter.messagebox as tkm
import rospy
import roslib
import actionlib
import time
import numpy as np
import math
import drogone_action.msg #import Servermessage
from std_srvs.srv import Empty # back_to_position_hold call service
from std_msgs.msg import Int8 #for communication between GUI and GUIServer
from std_msgs.msg import Float32MultiArray #for communication between GUI and Eight
from nav_msgs.msg import Odometry #for publishing point if detector is not working
# from libsbp_ros_msgs.msg import MsgAgeCorrections #msg published by piksi
# from piksi_rtk_msgs.msg import ReceiverState_V2_6_5 #msg published by piksi
from geometry_msgs.msg import PoseStamped #msg published by piksi



class GUIServer(object):
    #create message that is used to publish result
    result_ = drogone_action.msg.FSMResult()
    goal_ = drogone_action.msg.FSMGoal()
    #help variable
    done_ = 0
    #default velocity of ball
    ball_velocity = 5

    num_det_ = 0

    #define Constructor
    def __init__(self,name):
        self._action_name = name
        sub_1 = rospy.Subscriber("communication", Int8, self.callback, queue_size=10)
        # sub_piksi_age_of_corrections = rospy.Subscriber("/dragonfly/piksi/position_receiver_0/sbp/age_corrections)", MsgAgeCorrections, self.piksi_age_callback, queue_size=100)
        # sub_piksi_state = rospy.Subscriber("/dragonfly/piksi/position_receiver_0/ros/receiver_state", ReceiverState_V2_6_5 , self.piksi_state_callback, queue_size=100)
        sub_local = rospy.Subscriber("/dragonfly/mavros/local_position/pose", PoseStamped , self.position_callback, queue_size=100)
        self._as = actionlib.SimpleActionServer(self._action_name, drogone_action.msg.FSMAction, execute_cb=self.execute_cb, auto_start = False )
        self._as.start()

        # self.done_ = 0


    #define ActionServer execute_callback function
    def execute_cb(self, goal):

        current_state.set(goal.new_mode)

        #create subscriber to GUI msg
        # sub_1 = rospy.Subscriber("communication", Int8, self.callback, queue_size=1)

        if goal.new_mode == "Start":
            time.sleep(3.1); #time delay
            command_1.set("back_to_position_hold")
            command_2.set("no interaction possible")
            command_3.set("no interaction possible")
            # command_4.set("Publish eight, tilted around x-axis?")
            # command_5.set("Publish diagonal eight?")


        elif goal.new_mode == "Search":
            command_1.set("Ball found?")
            command_2.set("no interaction possible")
            command_3.set("Land")
            # command_3.set("Publish eight, tilted around y-axis?")
            # command_4.set("Publish eight, tilted around x-axis?")
            # command_5.set("Publish diagonal eight?")


        elif goal.new_mode == "Wait":
            # command_1.set("Ready for Follow/Catch?")
            command_1.set("Back to Search?")
            command_2.set("no interaction possible")
            command_3.set("Land")
            # command_3.set("no interaction possible")
            # command_4.set("no interaction possible")
            # command_5.set("no interaction possible")


        elif goal.new_mode == "Follow":
            command_1.set("Ball close enough?")
            # command_2.set("Ball lost?")
            command_2.set("no interaction possible")
            command_3.set("Land")
            # command_3.set("Publish eight with velocity:")
            # command_3.set("Publish eight, tilted around y-axis?")
            # command_4.set("Publish eight, tilted around x-axis?")
            # command_5.set("Publish diagonal eight?")

        elif goal.new_mode == "Catch":
            command_1.set("Ball caught?")
            command_2.set("Ball lost?")
            command_3.set("Land")
            # command_3.set("Publish eight, tilted around y-axis?")
            # command_4.set("Publish eight, tilted around x-axis?")
            # command_5.set("Publish diagonal eight?")

        # elif goal.new_mode == "FlyBack":
        #     command_1.set("Arrived at Drop Off position?")
        #     command_2.set("no interaction possible")
        #     command_3.set("no interaction possible")
        #     command_4.set("no interaction possible")
        #     command_5.set("no interaction possible")
        #
        # elif goal.new_mode == "Empty":
        #     command_1.set("Ball out of cage?")
        #     command_2.set("no interaction possible")
        #     command_3.set("no interaction possible")
        #     command_4.set("no interaction possible")
        #     command_5.set("no interaction possible")

        elif goal.new_mode == "Land":
            command_1.set("no interaction possible")
            command_2.set("no interaction possible")
            command_3.set("no interaction possible")
            # command_4.set("no interaction possible")
            # command_5.set("no interaction possible")

        else:
            command_1.set("no interaction possible")
            command_2.set("no interaction possible")
            command_3.set("no interaction possible")
            # command_4.set("no interaction possible")
            # command_5.set("no interaction possible")

        while (not self._as.is_preempt_requested()) and (not rospy.is_shutdown()) and (not self.done_):
            continue


        if (self._as.is_preempt_requested() or rospy.is_shutdown()):
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted(self.result_)

        elif self.done_==1 or (self.done_== 3 and goal.new_mode == "Catch"):
             rospy.loginfo('%s: Succeeded' % self._action_name)
             self._as.set_succeeded(self.result_)

        elif self.done_==2 or (self.done_==3 and not (goal.new_mode == "Catch")):
            rospy.loginfo('%s: Aborted' % self._action_name)
            self._as.set_aborted(self.result_)

        # sub_1.unregister()
        self.done_ = 0


    #define callback funciton of subscriber
    def callback(self, data):
        self.done_ = data.data
        # rospy.loginfo(self.done_)

    # def piksi_age_callback(self, data):
    #     age_of_correction.set(data)

    def position_callback(self, data):
        position.set(data.pose.position.x)


    # def piksi_state_callback(self, data):
    #     if data.rtk_mode_fix == 1:
    #         piksi_state.set("RTK Fixed")
    #     elif data.rtk_mode_fix == 0:
    #         piksi_state.set("RTK Float")
    #     num_sat.set(data.num_sat)


#callback function of button command
def command_1_callback():
    #help variable
    job_done = 0

    #check the command
    if command_1.get() == "back_to_position_hold":
        rospy.wait_for_service('/dragonfly/mavros/voliro/back_to_position_hold')
        back_to_position_hold = rospy.ServiceProxy('/dragonfly/mavros/voliro/back_to_position_hold', Empty)
        back_to_position_hold()
        command_1.set("Start")

    elif command_1.get() == "Start" or command_1.get() == "Ball found?" or command_1.get() == "Ball close enough?" or command_1.get() == "Ball caught?" or command_1.get() == "Ball out of cage?" or command_1.get() == "Arrived at Drop Off position?" or command_1.get() == "Ready for Follow/Catch?" or command_1.get() == "Restart?" or command_1.get() == "Back to Search?" :
        job_done = 1

    #-----------------------if publish & subscribes doesn't work at the first time------------------------------
    # while pub.get_num_connections() == 0:
    #     # rospy.loginfo("Waiting for subscriber to connect")
    #     rospy.sleep(0.1)
    #-----------------------------------------------------------------------------------------------------------
    #publish job status
    pub.publish(job_done)


def command_2_callback():
    #help variable
    job_done = 0

    if command_2.get() == "Ball lost?":
        job_done = 2

    # if command_2.get() == "Start without back_to_position_hold":
    #     job_done = 1

    #publish job status
    pub.publish(job_done)


def command_3_callback():
    #help variable
    job_done = 0

    if command_3.get() == "Land":
        command_3.set("Confirm Land")

    elif command_3.get() == "Confirm Land":
        job_done = 3

    #publish job status
    pub.publish(job_done)


    #
    # def command_4_callback():
    #     #help variable
    #     job_done = 0
    #
    #     if command_4.get() == "Publish eight, tilted around x-axis?":
    #         job_done = 4
    #
    #     #publish job status
    #     pub.publish(job_done)
    #
    #
    # def command_5_callback():
    #     #help variable
    #     job_done = 0
    #
    #     if command_5.get() == "Publish diagonal eight?":
    #         job_done = 5
    #
    #     #publish job status
    #     pub.publish(job_done)
    #
    # def ball_lost_1_callback():
    #     #publish job status
    #     duration = 5
    #     ball_lost_pub.publish(duration)
    #
    # def ball_lost_2_callback():
    #     #publish job status
    #     duration = 10
    #     ball_lost_pub.publish(duration)
    #
    # def ball_lost_3_callback():
    #     #publish job status
    #     duration = 15
    #     ball_lost_pub.publish(duration)
    #
    #
    # def ball_lost_callback():
    #     duration = int(input_1.get())
    #     # rospy.loginfo(duration)
    #     ball_lost_pub.publish(duration)
    #
    # def eight_parameters_callback():
    #     parameters = Float32MultiArray()
    #     parameters.data = [5, 0, 0, 10]
    #     #default values
    #     # parameters[0] = 12
    #     # parameters[1] = 5
    #     # parameters.data[2] = 38
    #     # parameters.data[3] = np.pi/36
    #
    #     #take entry values
    #     if not len(ball_velocity.get()) == 0:
    #         parameters.data[0] = int(ball_velocity.get())
    #         # rospy.loginfo(parameters.data[1])
    #     if not len(center_x.get()) == 0:
    #         parameters.data[1] = int(center_x.get())
    #         # rospy.loginfo(parameters.data[0])
    #     if not len(center_y.get()) == 0:
    #         parameters.data[2] = int(center_y.get())
    #         # rospy.loginfo(parameters.data[2])
    #     if not len(center_z.get()) == 0:
    #         parameters.data[3] = float(center_z.get())
    #
    #     parameters_pub.publish(parameters)




if __name__ == '__main__':
    rospy.init_node('MBZIRC_GUI')
    rospy.loginfo(rospy.get_name() + 'start')

    #Publsihers
    pub = rospy.Publisher("communication", Int8, queue_size=10)
    ball_lost_pub = rospy.Publisher("ball_lost_simulation", Int8, queue_size=10)
    parameters_pub = rospy.Publisher("parameters_eight", Float32MultiArray, queue_size=10)


    # # help variable
    # done = 0

    # initialize window
    window = Tk()
    window.title("GUI MBZIRC Challenge 1")
    #define size/geometr
    window.geometry('800x170')

    # frame surrounding other widgets
    frame = Frame(window)
    frame.grid()

    #current state
    Label(frame, text="Current State:", height = 2, width = 20).grid(row=0,column=0,sticky=W)
    #global variable
    current_state = StringVar()
    current_state.set("GUI is starting...")
    Label(frame, textvariable=current_state, height = 2, width = 20).grid(row=1,column=0,sticky=W)


    #Interaction with FSM
    Label(frame, text="Interaction:", height = 2, width = 20).grid(row=0,column=1,sticky=W)
    command_1 = StringVar()
    command_1.set("Waiting for interaction")
    Button(frame, textvariable=command_1, command=command_1_callback).grid(row=1, column=1)
    command_2 = StringVar()
    command_2.set("Waiting for interaction")
    Button(frame, textvariable=command_2, command=command_2_callback).grid(row=2, column=1)
    command_3 = StringVar()
    command_3.set("Waiting for interaction")
    Button(frame, textvariable=command_3, command=command_3_callback).grid(row=3, column=1)
    # command_4 = StringVar()
    # command_4.set("Waiting for interaction")
    # Button(frame, textvariable=command_4, command=command_4_callback).grid(row=4, column=1)
    # command_5 = StringVar()
    # command_5.set("Waiting for interaction")
    # Button(frame, textvariable=command_5, command=command_5_callback).grid(row=5, column=1)


    #RTK/Piksi Status:
    # Label(frame, text="RTK/Piksi Status:", height = 2, width = 20).grid(row=0,column=2,sticky=W)
    # Label_age_of_correction = StringVar()
    # Label_age_of_correction.set("age of correction:")
    # Label(frame, textvariable=Label_age_of_correction, height = 2, width = 30).grid(row=1,column=2,sticky=W)
    # age_of_correction = StringVar()
    # age_of_correction.set("-")
    # Label(frame, textvariable=age_of_correction, height = 2, width = 15).grid(row=1,column=3,sticky=W)
    # Label_state = StringVar()
    # Label_state.set("piksi state:")
    # Label(frame, textvariable=Label_state, height = 2, width = 30).grid(row=2,column=2,sticky=W)
    # piksi_state = StringVar()
    # piksi_state.set("-")
    # Label(frame, textvariable=piksi_state, height = 2, width = 15).grid(row=2,column=3,sticky=W)
    # Label_num_sat = StringVar()
    # Label_num_sat.set("number of satelites:")
    # Label(frame, textvariable=Label_num_sat, height = 2, width = 30).grid(row=3,column=2,sticky=W)
    # num_sat = StringVar()
    # num_sat.set("-")
    # Label(frame, textvariable=num_sat, height = 2, width = 15).grid(row=3,column=3,sticky=W)


    #Mavros Status
    Label(frame, text="Mavros Status:", height = 2, width = 20).grid(row=4,column=2,sticky=W)
    local_position = StringVar()
    local_position.set("local position x:")
    Label(frame, textvariable=local_position, height = 2, width = 30).grid(row=5,column=2,sticky=W)
    position = StringVar()
    position.set("-")
    Label(frame, textvariable=position, height = 2, width = 15).grid(row=5,column=3,sticky=W)


    #tracker tracking_status
    Label(frame, text="Tracking Status:", height = 2, width = 20).grid(row=0,column=2,sticky=W)
    tracker_now = StringVar()
    tracker_now.set("Tracker Cov Norm Now:")
    Label(frame, textvariable=tracker_now, height = 2, width = 30).grid(row=1,column=2,sticky=W)
    cov_now = StringVar()
    cov_now.set("-")
    Label(frame, textvariable=cov_now, height = 2, width = 15).grid(row=1,column=3,sticky=W)
    distance_ball = StringVar()
    distance_ball.set("Distance to ball:")
    Label(frame, textvariable=distance_ball, height = 2, width = 30).grid(row=2,column=2,sticky=W)
    distance = StringVar()
    distance.set("-")
    Label(frame, textvariable=distance, height = 2, width = 15).grid(row=2,column=3,sticky=W)
    detector_label = StringVar()
    detector_label.set("Number Detections:")
    Label(frame, textvariable=detector_label, height = 2, width = 30).grid(row=3,column=2,sticky=W)
    number = StringVar()
    number.set("-")
    Label(frame, textvariable=number, height = 2, width = 15).grid(row=3,column=3,sticky=W)

    #Simulation of ball lost
    # Label(frame, text="Simulate lost Ball:", height = 2, width = 20).grid(row=12,column=0,sticky=W)
    # input_1 = StringVar()
    # Entry(frame, textvariable = input_1).grid(row=12, column=1)
    # Enter_Duration = StringVar()
    # Enter_Duration.set("Enter Duration in 0.1sec, Commit")
    # Button(frame, textvariable=Enter_Duration, command=ball_lost_callback).grid(row=13, column=1)



    #tracking interruptions (simulate Ball lost)
    # ball_lost_1 = StringVar()
    # ball_lost_1.set("0.5sec lost")
    # Button(frame, textvariable=ball_lost_1, command=ball_lost_1_callback).grid(row=2, column=4)
    # ball_lost_2 = StringVar()
    # ball_lost_2.set("1sec lost")
    # Button(frame, textvariable=ball_lost_2, command=ball_lost_2_callback).grid(row=3, column=4)
    # ball_lost_3 = StringVar()
    # ball_lost_3.set("1.5sec lost")
    # Button(frame, textvariable=ball_lost_3, command=ball_lost_3_callback).grid(row=4, column=4)


    #initialize GUI Server class
    GUIServer = GUIServer("GUI")

    #display window till manually closed
    window.mainloop()

    rospy.spin() #rospyspin keeps node running
