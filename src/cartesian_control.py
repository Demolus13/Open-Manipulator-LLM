import rospy
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest

def move_end_effector(planning_group, end_effector_name, x, y, z, time):
    rospy.wait_for_service('/goal_task_space_path')
    try:
        move_service = rospy.ServiceProxy('/goal_task_space_path', SetKinematicsPose)
        request = SetKinematicsPoseRequest()
        request.planning_group = planning_group
        request.end_effector_name = end_effector_name
        request.kinematics_pose.pose.position.x = x
        request.kinematics_pose.pose.position.y = y
        request.kinematics_pose.pose.position.z = z
        request.kinematics_pose.pose.orientation.x = 0.0
        request.kinematics_pose.pose.orientation.y = 0.0
        request.kinematics_pose.pose.orientation.z = 0.0
        request.kinematics_pose.pose.orientation.w = 1.0
        request.path_time = time
        rospy.loginfo("Sending request to move end effector to x=%f, y=%f, z=%f over %f seconds" % (x, y, z, time))
        response = move_service(request)
        rospy.loginfo("Service response: %s" % response)
        return response.is_planned
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return False

if __name__ == "__main__":
    rospy.init_node('cartesian_control')
    planning_group = "arm"  # Replace with the correct planning group name if different
    end_effector_name = "gripper"
    x = 0.2  # Desired x coordinate
    y = -0.2  # Desired y coordinate
    z = 0.2  # Desired z coordinate
    time = 3.0  # Time to reach the desired coordinates
    success = move_end_effector(planning_group, end_effector_name, x, y, z, time)
    if success:
        rospy.loginfo("End effector moved successfully")
    else:
        rospy.logerr("Failed to move end effector")

# drop off x=0.124859, y=0.082899, z=0.041963