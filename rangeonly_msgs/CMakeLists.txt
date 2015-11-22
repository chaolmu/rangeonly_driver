cmake_minimum_required(VERSION 2.8.3)
project(rangeonly_msgs)

find_package(catkin REQUIRED
COMPONENTS
    roscpp
    rospy
    message_generation
    std_msgs
)

add_message_files(
    DIRECTORY msg
    FILES
    P2PRange.msg
)

generate_messages(DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        rospy
        std_msgs
        message_runtime
)
