# Copyright (c) 2023 Real-Time Development Center (RTDC) Project
# All rights reserved.
#!/usr/bin/env python3
# Simple ros script to verify publisher
import rospy
from nav_sensors.msg import DVL_MSG

def dvl_cb(dvl_msg):

    #log measurements of type dvl_msg:
    rospy.loginfo("Velocities %s", dvl_msg.data.velocity)
    rospy.loginfo("Velocity Error %s", dvl_msg.data.vel_error)
    rospy.loginfo("Beams %s", dvl_msg.data.beams)
    rospy.loginfo("Mean Bottom Range %s", dvl_msg.data.mean_bottom_range)
    rospy.loginfo("Speed of Sound %s", dvl_msg.data.speed_of_sound)
    rospy.loginfo("Input Voltage %s", dvl_msg.power.input_voltage)
    rospy.loginfo("Transmit Voltage %s", dvl_msg.power.transmit_voltage)
    rospy.loginfo("Transmit Current %s", dvl_msg.power.transmit_current)

    
def listener():

    # initialize node
    rospy.init_node('listener', anonymous=True)

    # create subscriber
    rospy.Subscriber("/hydrus/dvl", DVL_MSG, dvl_cb)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    # start subscriber
    listener()