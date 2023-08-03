#! /usr/bin/env python3

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
import PyKDL as kdl
import rospy
from sensor_msgs.msg import JointState
import random


base_link = "panda_link0"
end_link = "panda_link8"
robot_description = rospy.get_param('robot_description')
# URDF.from_xml_file('/home/blurryvm/catkin_ws/src/franka_description/robots/panda/panda.urdf')
robot_urdf = URDF.from_xml_string(robot_description)

kdl_tree = kdl_tree_from_urdf_model(robot_urdf)
chain = kdl_tree.getChain(base_link, end_link)
num_joints = chain.getNrOfJoints()


q_min = kdl.JntArray(num_joints)
q_max = kdl.JntArray(num_joints)
q_init = kdl.JntArray(num_joints)
q_out = kdl.JntArray(num_joints)

# Franka joint limits
q_min[0] = q_min[2] = q_min[4] = q_min[6] = -2.9
q_min[1] = -1.76
q_min[3] = -3.07
q_min[5] = -0.02

q_max[0] = q_max[2] = q_max[4] = q_max[6] = 2.9
q_max[1] = 1.76
q_max[3] = -0.07
q_max[5] = 3.75


def go_to_goal(chain, target_pos, q_init, q_min, q_max):
    '''
    Solves the robot kinematics problem to get to the eef target position
    '''
    target_pos = kdl.Vector(target_pos[0], target_pos[1], target_pos[2])
    target_orient = kdl.Rotation.RPY(0, 1.57, 0)  # in rads
    target_pos_frame = kdl.Frame(target_orient, target_pos)

    fk = kdl.ChainFkSolverPos_recursive(chain)
    ik_v = kdl.ChainIkSolverVel_pinv(chain)

    # We compute the inverse kinematics for position
    ik_p = kdl.ChainIkSolverPos_NR_JL(chain, q_min, q_max, fk, ik_v)

    q_out = kdl.JntArray(num_joints)
    # Compute the joint values that correspond to the target pose given our initial guess q_init.
    ik_p.CartToJnt(q_init, target_pos_frame, q_out)
    return q_out


def get_current_pos(chain, q_out):
    # Perform forward kinematics to get the EEF frame
    p_out = kdl.Frame()

    fk = kdl.ChainFkSolverPos_recursive(chain)
    fk.JntToCart(q_out, p_out)

    # get the current position of the EEF
    return round(p_out.p[0], 4), round(p_out.p[1], 4), round(p_out.p[2], 4)


def is_reachable(target_pos, current_pos):
    print()
    print("current_pos: ", current_pos, " target_pos:", target_pos)
    print("diff: ", (target_pos[0] - current_pos[0]) + (
        target_pos[1] - current_pos[1]) + (target_pos[2] - current_pos[2]))
    diff = (target_pos[0] - current_pos[0]) + (target_pos[1] -
                                               current_pos[1]) + (target_pos[2] - current_pos[2])
    if abs(diff) <= 5e-2:
        return True
    return False


# Now we can compute our needed joint angles to position our robot to reach the desired position.
# Before that, we set the initial position of the robot
q_init = kdl.JntArray(num_joints)
q_init[3] = -1.57

# Start up ROS and publish the joint states
rospy.init_node("go_to_goal")
joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
rate = rospy.Rate(10)

msg = JointState()
msg.name = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
            'panda_joint5', 'panda_joint6', 'panda_joint7']
msg.position = list(q_init)

start_time = rospy.Time.now()
while not rospy.is_shutdown():
    msg.header.stamp = rospy.Time.now()

    if rospy.Time.now() - start_time > rospy.Duration(2):
        # print("Published new state after 2 seconds!")

        # # Generate a new random target position (x, y, z)
        target_pos = (random.uniform(-0.4, 0.4),
                      random.uniform(-0.2, 0.3), random.uniform(-0.1, 1.15))

        q_out = go_to_goal(chain, target_pos, q_init, q_min, q_max)
        curr_pos = get_current_pos(chain, q_out)

        print(
            f"Target position x: {round(target_pos[0],2)}, y: {round(target_pos[1],2)}, z:{round(target_pos[2],2)}", end=" :--> ")

        if is_reachable(target_pos=target_pos, current_pos=curr_pos):

            q_init = q_out  # update the initial joint positions

            # convert q_out to list of Float values
            msg.position = list(q_out)
            print("REACHED!\n")
        else:
            print("NOT reachable\n")
            pass

        start_time = rospy.Time.now()

    joint_pub.publish(msg)
    rate.sleep()
