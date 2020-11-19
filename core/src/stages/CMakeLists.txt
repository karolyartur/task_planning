add_library(${PROJECT_NAME}_stages
	${PROJECT_INCLUDE}/stages/dummy.h

	dummy.cpp
)
target_link_libraries(${PROJECT_NAME}_stages ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME}_stages
	PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/stages>
	PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	PUBLIC ${catkin_INCLUDE_DIRS}
)
add_dependencies(${PROJECT_NAME}_stages ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}_stages
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
