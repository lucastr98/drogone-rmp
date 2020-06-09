#! /usr/bin/env python
import roslib
import rospy
import actionlib
import smach
import time
import smach_ros

from smach_ros import SimpleActionState
from drogone_action.msg import FSMAction, FSMGoal #import Servermessage


#--------------------------------------------------def termination callback functions-----------------------------------------------------------------------------------------------------------------------------------------

# gets called when ANY child state terminates and terminates all running states
def child_term_cb1(outcome_map):
    return 'True'

# gets called when ANY child state terminates (for Follow and Catch as Motionplanner is does not matter if it is finsihed)
def child_term_cb2(outcome_map):
    # terminates all running states if state GUI finished with outcome 'succeeded' or 'aborted'
    if outcome_map['GUI'] == 'succeeded' or outcome_map['GUI'] == 'aborted' or outcome_map['Check'] == 'succeeded':
        rospy.logwarn("true")
        return True
    # in all other case, just keep running, don't terminate anything (e.g. if Motion planner finished)
    else:
        rospy.logwarn("false")
        return False

# # gets called when ANY child state terminates
# def child_term_cb2(outcome_map):
#     # terminate all running states if state GUI finished with outcome 'succeeded' or 'aborted'
#     if outcome_map['Motionplanner'] == 'succeeded' or outcome_map['Motionplanner'] == 'aborted':
#         return 'True'
#     # in all other case, just keep running, don't terminate anything (e.g. if GUI finished)
#     else:
#         return 'False'
# -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

def follow_cb(outcome_map):
    if outcome_map['Check'] == 'succeeded' or outcome_map['GUI'] == 'succeeded':
        return 'Caught'
    elif outcome_map['Motionplanner'] == 'succeeded':
        return 'Followed'
    else:
        return 'Lost'

def catch_cb(outcome_map):
    if outcome_map['GUI'] == 'succeeded' or  outcome_map['Check'] == 'succeeded':
        return 'Caught'
    else:
        return 'Lost'




# helper function to create SimpleActionState according to FSM.action
#finalcmd is for now useless, maybe we find a usage for it,  leave it for now.
def create_action(server_name, mode):
    return SimpleActionState(server_name,
                             FSMAction,
                             goal = FSMGoal(new_mode=mode, finalcmd=1)
                             )


# main
def main():
    rospy.init_node('Finite_State_Machine')

    #wait for 2 secs until the rest started up
    rospy.sleep(2.0)

    # Create the toplevel SMACH state machine
    sm_top= smach.StateMachine(outcomes=['Finished'])


    # Configure Top Level state machine with states = open the container
    with sm_top:

        #--------------------------------------------------STATE: STARTMANUAL-----------------------------------------------------------------------------------------------------------------------------------------
        #START state waits for 3 secs for everything to start up, than in the GUI one can start everything
        sm_StartManual=smach.StateMachine(outcomes=['Started'])

        with sm_StartManual:
            smach.StateMachine.add('GUI',
                                    create_action('GUI', 'StartManual'),
                                    transitions={#'aborted':'Land',
                                                 'aborted':'Started',
                                                 'succeeded':'Started',
                                                 'preempted':'Started'})

        smach.StateMachine.add('StartManual', sm_StartManual,
                                transitions={'Started':'TakeOff'})



        #--------------------------------------------------STATE: TAKEOFF-----------------------------------------------------------------------------------------------------------------------------------------
        #Add State TakeOff such that Motionplanner creates takeoff trajectory
        #If Motionplanner succeeded as Drone arrived, state is finished
        sm_TakeOff=smach.Concurrence(outcomes=['TookOff', 'Failed'],
                                      default_outcome='TookOff',
                                      child_termination_cb = child_term_cb1,
                                      outcome_map = {'TookOff':{'Motionplanner':'succeeded'},
                                                     'Failed':{'Motionplanner':'aborted'}
                                                    })

        with sm_TakeOff:
            smach.Concurrence.add('Motionplanner', create_action('Motionplanner', 'TakeOff'))
            smach.Concurrence.add('GUI', create_action('GUI', 'TakeOff'))


        smach.StateMachine.add('TakeOff', sm_TakeOff,
                                transitions={'TookOff':'WaitforAutonomous',
                                             'Failed':'TakeOff'
                                             })



        #--------------------------------------------------STATE: WAITFORAUTONOMOUS-----------------------------------------------------------------------------------------------------------------------------------------
        #Add State WaitforAutonomous
        #In the GUI one can press Start autonomous to proceed with following the drone, this is the transition to autonomous flying
        sm_WaitforAutonomous=smach.StateMachine(outcomes=['Autonomous'])

        with sm_WaitforAutonomous:
            smach.StateMachine.add('GUI',
                                    create_action('GUI', 'WaitforAutonomous'),
                                    transitions={#'aborted':'Land',
                                                 'aborted':'Autonomous',
                                                 'succeeded':'Autonomous',
                                                 'preempted':'Autonomous'})

        smach.StateMachine.add('WaitforAutonomous', sm_WaitforAutonomous,
                                transitions={'Autonomous':'Follow'})



        #--------------------------------------------------STATE: FOLLOW-----------------------------------------------------------------------------------------------------------------------------------------
        #Add Concurent Follow such that Motionplanner creates a trajectory of flying underneath the victim drone and following the victim drone.
        #In the GUI one can press 'Catch' to proceed with catching the victim drone
        #In the GUI one can press 'Lost' if victim drone is lost
        sm_Follow=smach.Concurrence(outcomes=['Lost','Followed','Caught'],
                                    default_outcome='Followed',
                                    child_termination_cb = child_term_cb2,
                                    outcome_cb = follow_cb)

        # for now catch state is not being used and follow and catch are together in state follow. therefore we check for catch already in follow mode
        with sm_Follow:
            smach.Concurrence.add('Motionplanner',create_action('Motionplanner', 'Follow'))
            smach.Concurrence.add('GUI',create_action('GUI', 'Catch'))# GUi can be used to press victim drone caught if drone did not catch it but therefore a restart is possible
            smach.Concurrence.add('Check',create_action('Check', 'CheckforCatch')) #


        smach.StateMachine.add('Follow', sm_Follow,
                                transitions={'Lost':'WaitforAutonomous',
                                             'Followed':'Catch',
                                             'Caught':'WaitforInstruction'
                                             })


        #--------------------------------------------------STATE: CATCH-----------------------------------------------------------------------------------------------------------------------------------------
        #Add Concurent Catch such that Motionplanner creates a trajectory of flying to the position of the drone.
        #In the GUI one can press 'Caught' to proceed with landing the drone
        #In the GUI one can press 'Lost' if victim drone is lost
        #In Future: Dummy node for Simulation which checks if DroGone drone caught the victim drone
        sm_Catch=smach.Concurrence(outcomes=['Lost','Caught'],
                                    default_outcome='Caught',
                                    child_termination_cb = child_term_cb1,
                                    outcome_cb = catch_cb)

        with sm_Catch:
            smach.Concurrence.add('Motionplanner',create_action('Motionplanner', 'Catch'))
            smach.Concurrence.add('GUI',create_action('GUI', 'Catch'))
            smach.Concurrence.add('Check',create_action('Check', 'CheckforCatch'))

        smach.StateMachine.add('Catch', sm_Catch,
                                transitions={'Lost':'WaitforAutonomous',
                                             'Caught':'WaitforInstruction'
                                             })



        #--------------------------------------------------STATE: WAITFORINSTRUCTION-----------------------------------------------------------------------------------------------------------------------------------------
        #Add State WaitforInstruction, this is the end of autonomous flying
        #In the GUI one can press Restart to restart the whole Simulationus or one can press Land to land the drone
        sm_WaitforInstruction=smach.StateMachine(outcomes=['Restart', 'Land'])

        with sm_WaitforInstruction:
            smach.StateMachine.add('GUI',
                                    create_action('GUI', 'WaitforInstruction'),
                                    transitions={#'aborted':'Land',
                                                 'aborted':'Restart',
                                                 'succeeded':'Land',
                                                 'preempted':'Land'})

        smach.StateMachine.add('WaitforInstruction', sm_WaitforInstruction,
                                transitions={'Restart':'StartManual',
                                             'Land':'Land'
                                             })



        #--------------------------------------------------STATE: LAND-----------------------------------------------------------------------------------------------------------------------------------------
        #Add State Land such that Motionplanner creates a trajectory to land
        sm_Land=smach.Concurrence(outcomes=['Failed','Landed'],
                                  default_outcome='Landed',
                                  child_termination_cb = child_term_cb1,
                                  outcome_map = {'Failed':{'Motionplanner':'aborted'},
                                                 'Landed':{'Motionplanner':'succeeded'}
                                                 })

        with sm_Land:
            smach.Concurrence.add('Motionplanner', create_action('Motionplanner', 'Land'))
            smach.Concurrence.add('GUI', create_action('GUI', 'Land'))


        smach.StateMachine.add('Land', sm_Land,
                                transitions={'Failed':'Land',
                                             'Landed':'Finished'
                                             })



    #Create introspection server for Smach Viewer, Smach Viewer needs to be installed with executive_smach_visualization (github)
    sis = smach_ros.IntrospectionServer('Intro_Server', sm_top, '/SM_TOP')
    sis.start()

    # Execute SMACH plan
    outcome = sm_top.execute()

    rospy.spin() #rospyspin keeps node running

    sis.stop()


if __name__ == '__main__':
    main()
