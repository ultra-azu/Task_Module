#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from vnpy import *


__vn100_path = '/dev/ttyUSB0'
__vn100_baud_rate = 115200


vn100 = EzAsyncData.connect(__vn100_path, __vn100_baud_rate)


def talker():
    pub_y = rospy.Publisher('/hydrus/IMU/yaw', Float32, queue_size=10)
    pub_p = rospy.Publisher('/hydrus/IMU/pitch', Float32, queue_size=10)
    pub_r = rospy.Publisher('/hydrus/IMU/roll', Float32, queue_size=10)
    rospy.init_node('imu_driver')
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        value_x = vn100.next_data().yaw_pitch_roll.x;
        if(value_x < 0):
            value_x += 360
        value_y = vn100.next_data().yaw_pitch_roll.y;
        if(value_y < 0):
            value_y += 360
        value_z = vn100.next_data().yaw_pitch_roll.z;
        if(value_z < 0):
            value_z += 360

        pub_y.publish(value_x)
        pub_p.publish(value_y)
        pub_r.publish(value_z)
        
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass