import rospy
import smach
from your_package.update_pose_to_object import UpdatePoseToObjectState  # Import the UpdatePoseToObjectState class

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
        smach.StateMachine.add('UPDATE_POSE_TO_OBJECT_STATE', UpdatePoseToObjectState(move_group_name, object_topic, desired_object_name),
                               transitions={'success': 'ANOTHER_STATE',  # Replace with the name of the next state
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'})

        # Add other states and transitions as needed

    # Execute the state machine
    outcome = sm.execute()

if __name__ == '__main__':
    main()