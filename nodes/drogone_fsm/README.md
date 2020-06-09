This is the package for the Finite State Machine for DroGone.
Author: Felix Stadler
fstadler@ethz.ch

The dependencies are:
- executive_smach (for smach):
    git@github.com:ros/executive_smach.git
- executive_smach_visualization (for smach_viewer, optional to   view states during run process):
    git@github.com:k-okada/executive_smach_visualization.git
- catkin_simple:
    git@github.com:simonlynen/catkin_and_catkin_simple.git
- drogone_action
- sensor_msgs
- ros_comm_msgs:
    git@github.com:ros/ros_comm_msgs.git

# Finite State Machine:

To run GUI: launch GUI.launch in Node manager on localhost.

Structure of FSM for now: (can be controlled with GUI)
- 1) Start Manual (press in GUI)
- 2) Takeoff
- 3) Start autonomous flying (press in GUI) -> service back_to_position_hold is called
- 4) Follow
- 5) Catch
- 6) Wait for instruction (Land or Restart)
- 7) Land 

#Important changes to MBZIRC gate_one_fsm package:
- package name is finite_state_machine
- drogone_action is changed from GateOne.action to FSM.action
- victim drone position is now part of victim drone node/package and publishes to /victim_drone/odometry
