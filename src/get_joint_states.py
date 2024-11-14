#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

def joint_states_callback(msg):
    rospy.loginfo("Joint States: %s", msg)

def get_joint_states():
    rospy.init_node('get_joint_states', anonymous=True)
    rospy.Subscriber('/joint_states', JointState, joint_states_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        get_joint_states()
    except rospy.ROSInterruptException:
        pass