cmake_minimum_required(VERSION 3.0.2)
project(trajectory)

add_compile_options(-std=c++11)

find_package (Eigen3 3.3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
)

# find_package(Boost REQUIRED COMPONENTS system)




## Generate added messages and services with any dependencies listed here
generate_messages( DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
)


catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf visualization_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(trajectory src/trajectory.cpp)
target_link_libraries(trajectory ${catkin_LIBRARIES} Eigen3::Eigen)
add_dependencies(trajectory ${catkin_EXPORTED_TARGETS} ${_EXPORTED_TARGETS})

add_executable(wayPoints src/wayPoints.cpp)
target_link_libraries(wayPoints ${catkin_LIBRARIES})
add_dependencies(wayPoints ${catkin_EXPORTED_TARGETS} ${_EXPORTED_TARGETS})

add_executable(testEnvironment src/testEnvironment.cpp)
target_link_libraries(testEnvironment ${catkin_LIBRARIES})
add_dependencies(testEnvironment ${catkin_EXPORTED_TARGETS} ${_EXPORTED_TARGETS})

