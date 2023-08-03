# franka_kinematics_example

sudo apt-get install ros-noetic-urdfdom-py
sudo apt-get install ros-noetic-franka-gazebo
pip install urdf-parser-py

git clone https://kwasiboatjr@bitbucket.org/traclabs/trac_ik.git
```
roslaunch franka_gazebo panda.launch use_gripper:=true controller:=effort_joint_trajectory_controller
```
