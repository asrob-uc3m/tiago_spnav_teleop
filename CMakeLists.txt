cmake_minimum_required(VERSION 3.0)

project(tiago_spnav_teleop LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
    actionlib
    play_motion_msgs
    sensor_msgs
    controller_manager_msgs
    controller_interface
    hardware_interface
    pluginlib
    roscpp
    kdl_parser
    urdf
)

find_package(orocos_kdl REQUIRED)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/tiago_spnav_teleop.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir config launch scripts)
    install(DIRECTORY ${dir}
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
endforeach()

install(PROGRAMS scripts/init_spnav.py
                 scripts/stop_spnav.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES tiago_spnav_teleop_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
