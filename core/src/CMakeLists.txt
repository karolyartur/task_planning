add_subdirectory(stages)

add_executable(interactive_task interactive_task.cpp)
target_link_libraries(interactive_task
  ${PROJECT_NAME}_stages
  ${catkin_LIBRARIES})

install(TARGETS interactive_task
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ${PROJECT_INCLUDE}/stages/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
