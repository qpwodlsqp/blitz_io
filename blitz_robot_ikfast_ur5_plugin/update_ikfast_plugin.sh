rosrun moveit_kinematics create_ikfast_moveit_plugin.py --search_mode=OPTIMIZE_MAX_JOINT --srdf_filename=blitz_robot.srdf --robot_name_in_srdf=blitz_robot --moveit_config_pkg=blitz_robot_moveit_config blitz_robot ur5 blitz_robot_ikfast_ur5_plugin arm_base_link arm_wrist_3_link /home/fetch/catkin_ws/src/blitz_robot_ikfast_ur5_plugin/src/blitz_robot_ur5_ikfast_solver.cpp