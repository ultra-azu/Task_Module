import rospy
import smach
import smach_ros
import actionlib

from action_controller.msg import MultiDofFollowJointTrajectoryAction, MultiDofFollowJointTrajectoryGoal
from geometry_msgs.msg import Transform, Vector3


class MoveRobot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=['trajectory', 'orientations'])

        self.client = actionlib.SimpleActionClient(
            'multi_dof_joint_trajectory_action', MultiDofFollowJointTrajectoryAction)

    def execute(self, userdata):
        if len(userdata.trajectory) != len(userdata.orientations):
            rospy.logerr('Number of points in trajectory and orientations do not match')
            return 'aborted'

        goal = MultiDofFollowJointTrajectoryGoal()
        goal.trajectory.joint_names.append('Base')

        for i in range(len(userdata.trajectory)):
            transform = Transform()
            transform.translation = Vector3(userdata.trajectory[i][0], userdata.trajectory[i][1], userdata.trajectory[i][2])
            transform.rotation.x = userdata.orientations[i][0]
            transform.rotation.y = userdata.orientations[i][1]
            transform.rotation.z = userdata.orientations[i][2]
            transform.rotation.w = userdata.orientations[i][3]

            goal.trajectory.points.append(MultiDofFollowJointTrajectoryGoal().trajectory.points.append(transform))

        self.client.wait_for_server()

        self.client.send_goal(goal)

        self.client.wait_for_result()

        result = self.client.get_state()

        if result == actionlib.GoalStatus.SUCCEEDED:
            return 'succeeded'
        else:
            return 'aborted'
        

def main():
    rospy.init_node('move_robot')

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    with sm:
        smach.StateMachine.add('MOVE_ROBOT', MoveRobot(), 
                               transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    sis.stop()

    rospy.spin()


if __name__ == '__main__':
    main()