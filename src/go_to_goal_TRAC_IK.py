
#! /usr/bin/env python3

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import *
from trac_ik_python.trac_ik import IK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rospy


def go_to_goal(pose, q_init=None):
    print("Target pose:", pose)
    pos, quat = PoseConv.to_pos_quat(pose)
    solution = ik.get_ik(q_init, pos[0], pos[1],
                         pos[2], quat[0], quat[1], quat[2], quat[3])

    print("Solution found: ", True if solution is not None else False)
    return solution


robot_description = rospy.get_param('robot_description')
robot_urdf = URDF.from_xml_string(robot_description)
base_link = "panda_link0"
end_link = "panda_link8"

robot_description = rospy.get_param('robot_description')
ik = IK('panda_link0', 'panda_link8', urdf_string=robot_description)

print(ik.joint_names)
num_joints = ik.number_of_joints
print(num_joints)

# Now we can compute our needed joint angles to position our robot to reach the desired position.
# Before that, we set the initial position of the robot
q_init = [0.0]*7
q_init[3] = -1.57

# Start up ROS and publish the joint states
rospy.init_node("go_to_goal")
joint_pub = rospy.Publisher(
    '/effort_joint_trajectory_controller/command', JointTrajectory, queue_size=10)
rate = rospy.Rate(10)

trajectory_msg = JointTrajectory()
trajectory_msg.points.append(JointTrajectoryPoint())

# Target poses to reach
target_poses = [[[0.0, 2.4, 0.8], [0.0, 0.3, 1.0]],  # Should fail
                [[0.0, 0.4, 0.4],  [0.0, 0.0, 1.0]],
                [[0, -0.4, 0.4], [0.0, 0.0, 1.0]],
                [[0, -0.4, 0.8], [1.57, 0.0, 1.0]],
                ]

while not rospy.is_shutdown():
    for target_pose in target_poses:

        trajectory_msg.header.stamp = rospy.Time.now()

        # compute the future joint angles using inverse kinematics
        q_out = go_to_goal(target_pose, q_init)

        if q_out is not None:
            trajectory_msg.joint_names = list(ik.joint_names)
            trajectory_msg.points[0].positions = q_out
            trajectory_msg.points[0].time_from_start = rospy.Duration(3)
            joint_pub.publish(trajectory_msg)
            rospy.sleep(4)
            print("REACHED!\n")

            # update the initial guess positions with current joint positions
            q_init = q_out

        else:
            print("NOT reachable\n")

        rate.sleep()
