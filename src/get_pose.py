from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
import numpy as np
import PyKDL as kdl
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import random
import math


def get_current_pos(chain: kdl.Chain, q_out: kdl.JntArray):
    # Perform forward kinematics to get the EEF frame
    p_out = kdl.Frame()
    fk = kdl.ChainFkSolverPos_recursive(chain)
    fk.JntToCart(q_out, p_out)

    # get the current position of the EEF
    print(f"x={round(p_out.p[0], 2)}, y={round(p_out.p[1],2)}, z={round(p_out.p[2],2)}")

def joints_cb(msg):
    global q_out
    global chain
    for i in range(len(msg.position)):
        q_out[i] = msg.position[i]

    get_current_pos(chain, q_out)

base_link = "panda_link0"
end_link = "panda_link8"
robot_urdf = URDF.from_xml_file('kdl_franka/urdf/panda.urdf')

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

rospy.init_node("get_current_pos")
rospy.Subscriber("/joint_states", JointState, callback=joints_cb)
rospy.spin()
