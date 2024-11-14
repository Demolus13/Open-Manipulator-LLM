#!/usr/bin/env python

import rospy
from open_manipulator_msgs.msg import KinematicsPose

def kinematics_pose_callback(data):
    position = data.pose.position
    orientation = data.pose.orientation
    rospy.loginfo("End effector coordinates: x=%f, y=%f, z=%f" % (position.x, position.y, position.z))
    rospy.loginfo("End effector orientation: x=%f, y=%f, z=%f, w=%f" % (orientation.x, orientation.y, orientation.z, orientation.w))

if __name__ == "__main__":
    rospy.init_node('get_end_effector_coordinates', anonymous=True)
    rospy.Subscriber("/gripper/kinematics_pose", KinematicsPose, kinematics_pose_callback)
    rospy.spin()

# upper right: End effector coordinates: x=-0.015955, y=0.152749, z=0.031111
# upper left: End effector coordinates: x=-0.020716, y=-0.149990, z=0.033823
# bottom left: End effector coordinates: x=0.264617, y=-0.152468, z=0.046600
# bottom right: End effector coordinates: x=0.272969, y=0.142075, z=0.041914