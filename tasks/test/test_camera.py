#!/usr/bin/env python
import sys
sys.path.append("/root/catkin_ws/src/Task_Module")
import unittest
import rospy
import os
import rosbag
import yaml
import unittest
import rospy
from tasks.src.data import shared_data, initialize_subscribers
class Camera_Behaviours(unittest.TestCase):
    def setUp(self):

        rospy.init_node('test_camera_node', anonymous=True)
        file_path = os.path.join(os.path.dirname(__file__), 'test_data/test_topics.yml')
        initialize_subscribers(file_path)
        # sm = YourStateMachine()
        # outcome = sm.execute()

    def test_camera_input(self):
        print("Testing Camera Input")
        self.assertIsNotNone(shared_data.zed_data['ObjectsStamped'])
        self.assertIsNotNone(shared_data.zed_data['RGBDSensors'])
        self.assertIsNotNone(shared_data.dvl_data)

    def tearDown(self):
        # Clean up code here
        pass

if __name__ == '__main__':
    import rostest
    rostest.rosrun('tasks', 'test_camera_behaviours', Camera_Behaviours)
