import sys
import rospy
from open_manipulator_msgs.msg import KinematicsPose
from sensor_msgs.msg import JointState

def kinematics_pose_callback(data):
    position = data.pose.position
    orientation = data.pose.orientation
    sys.stdout.write("\rEnd effector: [%f, %f, %f, %f, %f, %f, %f]" % (position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w))
    sys.stdout.flush()

def joint_state_callback(data):
    joint_positions = data.position
    sys.stdout.write("\rJoint positions: [%f, %f, %f, %f]" % (joint_positions[0], joint_positions[1], joint_positions[2], joint_positions[3]))
    sys.stdout.flush()

if __name__ == "__main__":
    rospy.init_node('get_end_effector_coordinates', anonymous=True)
    rospy.Subscriber("/gripper/kinematics_pose", KinematicsPose, kinematics_pose_callback)
    rospy.Subscriber("/joint_states", JointState, joint_state_callback)
    rospy.spin()