file(GLOB LISTENER_HEADERS *.h)
file(GLOB LISTENER_SOURCES *.cpp *.c)


add_executable(listener_node ${LISTENER_SOURCES} ${LISTENER_HEADERS})

target_link_libraries(listener_node ${catkin_LIBRARIES})

install(TARGETS listener_node DESTINATION ./bin)
