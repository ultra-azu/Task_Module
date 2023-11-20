# This is the example state machine  for
# constrcuting the others machines with their respective 
# missions.

from data import shared_data, initialize_subscribers
import smach
import rospy
from movement import UpdatePoseState, UpdatePoseToObjectState


"""
--------------------------------------------
DEFINE YOUR CUSTOM STATES IN HERE IF NECESSARY
--------------------------------------------

class CustomState(smach.StateMachine):
    def __init__(self):
        pass

"""






class YourStateMachine(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success', 'failure'])
        self.userdata.shared_data = shared_data

        with self:
            smach.StateMachine.add('UPDATE_POSE', UpdatePoseState(...), transitions={'success':'ANOTHER_STATE', 'aborted':'failure', 'preempted':'failure'}, remapping={'camera_data_in':'shared_data'})

# Running the state machine
if __name__ == '__main__':
    rospy.init_node('your_state_machine_node')
    initialize_subscribers()
    sm = YourStateMachine()
    outcome = sm.execute()
