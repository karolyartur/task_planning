cmake_minimum_required(VERSION 2.8.12)
project(task_planning_core)

add_compile_options(-std=c++14)

find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
	roscpp
	moveit_core
	moveit_task_constructor_core
  moveit_ros_planning
	moveit_ros_planning_interface
	rosparam_shortcuts
  moveit_task_constructor_msgs
  task_planning_msgs
  visualization_msgs
  rviz_marker_tools
)

catkin_package(
	LIBRARIES
		${PROJECT_NAME}_stages
	INCLUDE_DIRS
		include
	CATKIN_DEPENDS
		geometry_msgs
		moveit_core
		moveit_task_constructor_msgs
        task_planning_msgs
		visualization_msgs
)

set(PROJECT_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)

catkin_install_python(PROGRAMS scripts/add_pick_action_client.py scripts/plan_action_client.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
