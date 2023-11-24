# This is the example state machine  for
# constructing the others machines with their respective 
# missions.

from tasks.src.data import shared_data, initialize_subscribers
import smach
import rospy
from tasks.src.movement import UpdatePoseState, UpdatePoseToObjectState


"""
--------------------------------------------
DEFINE YOUR CUSTOM STATES IN HERE IF NECESSARY
--------------------------------------------
"""
# Define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['try_again'])

    def execute(self, userdata):
        print("Try Again to detect object")
        return "try_again"
    




class YourStateMachine(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success', 'failure'])
        # shared_data is initialized inisde the initialize_subscribers() function
        # This is a global variable that is shared between all the states.
        self.userdata.shared_data = shared_data

        with self:
            smach.StateMachine.add('move_to_object', UpdatePoseToObjectState(), transitions={'success':'ANOTHER_STATE', 'aborted':'failure', 'edge_cade_detected':'FOO', "object_not_detected":"FOO"})
            smach.StateMachine.add('FOO', Foo(), transitions={'success':'ANOTHER_STATE', 'aborted':''})

# Running the state machine
if __name__ == '__main__':
    rospy.init_node('your_state_machine_node')
    initialize_subscribers()
    sm = YourStateMachine()
    outcome = sm.execute()
