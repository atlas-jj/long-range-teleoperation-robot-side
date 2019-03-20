cmake_minimum_required(VERSION 2.8.3)
project(teleop_wam_baseline)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs message_generation std_msgs
moveit_core moveit_ros_planning moveit_ros_planning_interface pluginlib cmake_modules barrett_wam_moveit_config
wam_msgs wam_srvs
)
find_package(Boost REQUIRED system filesystem date_time thread)

find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(SDL REQUIRED)
set(LIBS ${SDL_LIBRARY})
add_message_files(
    FILES
        Key.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
CATKIN_DEPENDS  roscpp std_msgs message_runtime
DEPENDS ${LIBS}
moveit_core
moveit_ros_planning_interface
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})

include_directories ("${PROJECT_SOURCE_DIR}/include")
add_subdirectory ("${PROJECT_SOURCE_DIR}/lib")

include_directories(
include
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${SDL_INCLUDE_DIR}
)

## Declare a C++ executable
add_executable(tele_op src/main.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(tele_op
  ${LIBS}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  #${PCL_LIBRARIES}
)
target_link_libraries (tele_op tele_lib )
