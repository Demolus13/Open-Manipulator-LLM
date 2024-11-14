import time
import rospy
from sensor_msgs.msg import JointState
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest

def move_end_effector(x, y, z, q1=0.0, q2=0.0, q3=0.0, q4=0.0, path_time=3.0):
    rospy.wait_for_service('/goal_task_space_path')
    try:
        move_service = rospy.ServiceProxy('/goal_task_space_path', SetKinematicsPose)
        request = SetKinematicsPoseRequest()
        request.planning_group = "arm"
        request.end_effector_name = "gripper"
        request.kinematics_pose.pose.position.x = x
        request.kinematics_pose.pose.position.y = y
        request.kinematics_pose.pose.position.z = z
        request.kinematics_pose.pose.orientation.x = q1
        request.kinematics_pose.pose.orientation.y = q2
        request.kinematics_pose.pose.orientation.z = q3
        request.kinematics_pose.pose.orientation.w = q4
        request.path_time = path_time
        rospy.loginfo("Sending request to move end effector to x=%f, y=%f, z=%f over %f seconds" % (x, y, z, path_time))
        response = move_service(request)
        rospy.loginfo("Service response: %s" % response)

        time.sleep(path_time)
        return response.is_planned
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return False

if __name__ == '__main__':
    rospy.init_node('move_end_effector_node')
    try:
        # home_positions = [0.170184, -0.017297, 0.086728, 0.036180, 0.663726, -0.040664, 0.745993]

        joint_positions = [0.222343, -0.112846, 0.037686, 0.157060, 0.624986, -0.186369, 0.741614]
        # joint_positions = [0.244503, 0.099244, 0.034381, -0.124351, 0.608074, 0.157093, 0.768183]
        move_end_effector(*joint_positions)
    except rospy.ROSInterruptException:
        pass