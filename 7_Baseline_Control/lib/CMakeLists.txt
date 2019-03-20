find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs cv_bridge std_msgs image_transport message_generation)
#find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(SDL REQUIRED)
set(LIBS ${SDL_LIBRARY})

catkin_package(
  CATKIN_DEPENDS  roscpp std_msgs message_runtime
  DEPENDS ${LIBS}
)
find_package(Eigen REQUIRED)
include_directories(
include
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${SDL_INCLUDE_DIR}
)


add_library (
tele_lib
math_helper.cpp
string_convertor.cpp
colormod.cpp
keyboard.cpp
)

target_link_libraries(tele_lib
  ${LIBS}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  #${PCL_LIBRARIES}
)
