cmake_minimum_required(VERSION 2.8.3)
project(explorer_moveit)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    geometric_shapes
    moveit_visual_tools
    roscpp
    explorer_msgs
    tf
   
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)



###########
## Build ##
###########

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(explorer_moveit src/explorer_moveit.h src/explorer_moveit.cpp)

target_link_libraries(explorer_moveit ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(explorer_moveit explorer_msgs_generate_messages)
