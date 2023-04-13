import rospy
import smach
from movement import * # Import the UpdatePoseToObjectState class
import utils

 # Import the CheckImageVisibleState class
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

class CheckImageVisibleState(smach.State):
    def __init__(self, image_topic, desired_object_name):
        smach.State.__init__(self, outcomes=['undetected', 'detected', 'preempted'])
        self.image_data = None
        self.image_topic = image_topic
        self.desired_object_name = desired_object_name

        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber(self.image_topic, self.image_callback)

        def execute(self, userdata):
            if self.image_data is not None:
                if self.desired_object_name in self.image_data.objects:
                    return 'detected'
                else:
                    return 'undetected'
            else:
                return 'preempted'


    with sm:
        # Fix the Todo in lower depth state
        smach.StateMachine.add('Lower_Depth', LowerDepth(move_group_name),s
                               transitions={'success': 'CHECK_IMAGE_VISIBLE',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'},
        smach.StateMachine.add('CHECK_IMAGE_VISIBLE', CheckImageVisibleState(image_topic, desired_object_name),
                               transitions={'undetected': 'Rotate90DegreesState',
                                            'detected': 'Survey_Gate',
                                            'preempted': 'preempted'},)
        smach.StateMachine.add('Rotate90DegreesState', Rotate90DegreesState(move_group_name),
                                 transitions={'success': 'CHECK_IMAGE_VISIBLE',
                                              'aborted': 'aborted',
                                              'preempted': 'preempted'},)
        
        smach.StateMachine.add('SURVEY_GATE', UpdatePoseToObjectState(move_group_name, object_topic, 'Gate'),
                                 transitions={'success': 'completed',
                                              'aborted': 'aborted',
                                              'preempted': 'preempted'},)

    # Execute the state machine
    outcome = sm.execute()

if __name__ == '__main__':
    main()