cmake_minimum_required(VERSION 2.8.12)
project(task_planning_msgs)

set(MSG_DEPS moveit_msgs moveit_task_constructor_msgs visualization_msgs)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	${MSG_DEPS}
)

# ROS messages, services and actions

add_action_files(DIRECTORY action FILES
	AddPick.action
	AddPlace.action
	AddRelease.action
	ControlTask.action
)

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(
	CATKIN_DEPENDS
		message_runtime
		${MSG_DEPS}
)
