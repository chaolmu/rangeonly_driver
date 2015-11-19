cmake_minimum_required(VERSION 2.8.3)
project(nanotron_swarm)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Release)

if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set (CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/bin" CACHE PATH "default install path" FORCE )
endif()

find_package(catkin REQUIRED
COMPONENTS
    roscpp
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
        std_msgs
        message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} 
		src/main.cpp 
		src/cntronbase.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
