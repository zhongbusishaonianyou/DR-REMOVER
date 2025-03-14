 cmake_minimum_required(VERSION 2.8.3)
project(remover)
set(CMAKE_CONFIGURATION_TYPES "Release" CACHE STRING "" FORCE)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    message_generation
    std_msgs
    nav_msgs
    roslaunch
    cv_bridge
    pcl_conversions
    pcl_ros
    geometry_msgs
    laser_geometry
    sensor_msgs
)

add_message_files(
   FILES
   node.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)


find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)


catkin_package(
  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  src
  include
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

# Online Map refinement
set(remover_src
        src/offline_map_updater/src/map_updater.cpp
        src/offline_map_updater/src/map_remover.cpp
        src/offline_map_updater/src/remover_utils.cpp
)

add_executable(visualize_map src/utils/visualize_map.cpp src/offline_map_updater/src/remover_utils.cpp)
target_link_libraries(visualize_map ${catkin_LIBRARIES})
add_dependencies(visualize_map remover_generate_messages_cpp)

add_executable(offline_map_updater src/offline_map_updater/main.cpp ${remover_src})
target_link_libraries(offline_map_updater ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(offline_map_updater remover_generate_messages_cpp)


