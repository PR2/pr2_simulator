#!/usr/bin/tcsh

echo "----------------starting roscore"
$ROS_ROOT/bin/roscore &

sleep 5

echo "----------------starting gazebo"
source `rospack find gazebo`/setup.tcsh
gazebo `rospack find gazebo_worlds`/worlds/empty.world &

sleep 5


echo "----------------roslaunch xml"
`rospack find xacro`/xacro.py `rospack find pr2_defs`/robots/r_arm.xacro.xml > pr2_arm.xml
python ./setparam.py

echo "----------------urdf2factory"
`rospack find gazebo_tools`/urdf2factory "robot_description"

echo "----------------spawn controller"
`rospack find mechanism_control`/scripts/mech.py sp `rospack find pr2_arm_gazebo`/r_arm_default_controller.xml

echo "----------------set gripper gap"
`rospack find robot_mechanism_controllers`/scripts/control.py set r_gripper_controller 0.0


