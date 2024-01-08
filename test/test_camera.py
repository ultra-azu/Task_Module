# This file is for testing the Camera inputs
# We used a ros-bag for imitating the cameras ouput

import sys
sys.path.append("/root/catkin_ws/src/TASK_MODULE")
import unittest
import rospy 
import os
import smach
from tasks.src.data import initialize_subscribers
from tasks.src.example import YourStateMachine
import rosbag


# class Camera_Behaviours(unittest.TestCase):
#     def setUp(self):
#         rospy.init_node('your_state_machine_node')
#         file_path = os.path.join(os.path.dirname(__file__), '/test_data/test_topics.yml')
#         initialize_subscribers(file_path)
#         sm = YourStateMachine()
#         outcome = sm.execute()


#     def checkCameraInput():
#         pass


# if __name__ == '__main__':
#     import rostest
#     rostest.rosrun('your_package', 'test_multidof6_state_machine', Camera_Behaviours)