rosrun moveit_kinematics create_ikfast_moveit_plugin.py --search_mode=OPTIMIZE_MAX_JOINT --srdf_filename=ovis.srdf --robot_name_in_srdf=ovis --moveit_config_pkg=ovis_moveit_config ovis manipulator ovis_manipulator_ikfast_plugin ovis_link_base ovis_end_effector /home/alex/Code/ovis_ws/src/ovis/ovis_description/urdf/ovis_manipulator_ikfast_plugin/src/ovis_manipulator_ikfast_solver.cpp