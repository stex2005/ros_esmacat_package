file(GLOB MY_PROJECT_HEADERS *.h)
file(GLOB MY_PROJECT_SOURCES *.cpp *.c)

add_executable(my_project ${MY_PROJECT_SOURCES})
target_link_libraries(my_project ethercat_driver esmacat   ${catkin_LIBRARIES})

install(TARGETS my_project DESTINATION ./bin)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
