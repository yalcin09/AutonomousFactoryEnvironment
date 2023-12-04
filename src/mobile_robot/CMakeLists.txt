cmake_minimum_required(VERSION 2.8.3)

project(mobile_robot)
set(CMAKE_CXX_STANDARD 11)

find_package (Eigen3 3.3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS 
  LIBRARIES 
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs
    geometry_msgs
    nav_msgs
    tf  
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

find_package(roslaunch)

foreach(dir config launch meshes urdf src)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

#add_executable(odometry_node src/odometry.cpp)
#target_link_libraries(odometry_node   ${catkin_LIBRARIES} Eigen3::Eigen)
#add_dependencies(odometry_node ${catkin_EXPORTED_TARGETS} ${_EXPORTED_TARGETS})



add_executable(odometryNode src/ekfOdometry.cpp)
target_link_libraries(odometryNode   ${catkin_LIBRARIES} Eigen3::Eigen)
add_dependencies(odometryNode ${catkin_EXPORTED_TARGETS} ${_EXPORTED_TARGETS})


add_executable(controllerNode src/controller.cpp)
target_link_libraries(controllerNode   ${catkin_LIBRARIES} Eigen3::Eigen)
add_dependencies(controllerNode ${catkin_EXPORTED_TARGETS} ${_EXPORTED_TARGETS})
