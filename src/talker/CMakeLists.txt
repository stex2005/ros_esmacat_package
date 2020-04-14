file(GLOB TALKER_HEADERS *.h)
file(GLOB TALKER_SOURCES *.cpp *.c)


add_executable(talker_node ${TALKER_SOURCES} ${TALKER_HEADERS})

target_link_libraries(talker_node ${catkin_LIBRARIES})

install(TARGETS talker_node DESTINATION ./bin)
