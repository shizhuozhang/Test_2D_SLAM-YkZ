cmake_minimum_required(VERSION 2.8.3)
project(myodom)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
)

catkin_package(
  # INCLUDE_DIRS include
#  LIBRARIES base_controller
CATKIN_DEPENDS roscpp 
#  DEPENDS system_lib
)


# find_package(catkin REQUIRED COMPONENTS nav_msgs roscpp tf)

# catkin_package(
# #  INCLUDE_DIRS include
# #  LIBRARIES base_controller
#  roscpp
# #  DEPENDS system_lib
# )

 include_directories( ${catkin_INCLUDE_DIRS})

add_executable(myodom
  src/myodom.cpp)
add_dependencies(myodom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(myodom
  ${catkin_LIBRARIES})