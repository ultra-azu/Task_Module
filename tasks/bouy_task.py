import rospy
import smach
from movement import * # Import the UpdatePoseToObjectState class
import utils



def main():
    rospy.init_node('multidof6_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['success', 'aborted', 'preempted'])

    # Define the desired object name
    desired_object_name = "target_object"

    # Define the object topic name
    object_topic = "/your_object_topic"  # Replace with the actual topic name

    # Define the MoveIt! move group
    move_group_name = "move_group"


    with sm:
        smach.StateMachine.add("Check_Bouy", utils.CheckImageVisibleState(object_topic, "Abyss"),
                               transitions={'undetected': 'aborted',
                                            'detected': 'Follow_Path',})
        smach.StateMachine.add('Go_to_Buoy', UpdatePoseToObjectState(move_group_name, object_topic, "Abyss"),
                               transitions={'success': 'success',
                                            'aborted': 'aborted',
                                            'preempted': 'Check_Path_Pinger'},)
    # Execute the state machine
    outcome = sm.execute()

if __name__ == '__main__':
    main()