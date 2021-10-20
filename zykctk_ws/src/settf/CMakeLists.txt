cmake_minimum_required(VERSION 2.8.3)
project(settf)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  geometry_msgs
)

catkin_package(
  # INCLUDE_DIRS include
#  LIBRARIES base_controller
CATKIN_DEPENDS roscpp 
#  DEPENDS system_lib
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
# ${catkin_INCLUDE_DIRS}
)



add_executable(tf_broadcaster
  src/tf_broadcaster.cpp
)
add_dependencies(tf_broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_broadcaster
  ${catkin_LIBRARIES}
)