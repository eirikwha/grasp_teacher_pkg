cmake_minimum_required(VERSION 2.8.3)
project(grasp_teacher)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
        rospy
        std_msgs
        sensor_msgs
        geometry_msgs
        grasp_teacher_msgs
        )

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
        scripts/test.py
        scripts/grasp_teacher_node.py
        scripts/pose_recorder_client.py
        scripts/pose_recorder_server.py
        scripts/grasp_teacher_client.py
        scripts/grasp_teacher_server.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )