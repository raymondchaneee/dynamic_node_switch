cmake_minimum_required(VERSION 2.8.3)
project(dynamic_node_switch)


find_package(catkin REQUIRED COMPONENTS rospy dynamic_reconfigure)

generate_dynamic_reconfigure_options(
    cfg/NodeSwitch.cfg
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
    scripts/dynamic_node_switch_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
