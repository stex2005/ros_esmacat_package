file(GLOB INTERFACE_HEADERS *.h)
file(GLOB INTERFACE_SOURCES *.cpp *.c)


add_executable(INTERFACE_node ${INTERFACE_SOURCES} ${INTERFACE_HEADERS})

target_link_libraries(INTERFACE_node ${catkin_LIBRARIES})

install(TARGETS INTERFACE_node DESTINATION ./bin)
