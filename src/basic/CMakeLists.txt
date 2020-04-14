file(GLOB CHATTER_HEADERS *.h)
file(GLOB CHATTER_SOURCES *.cpp *.c)


add_executable(basic_node ${CHATTER_SOURCES} ${CHATTER_HEADERS})

target_link_libraries(basic_node ${catkin_LIBRARIES})

install(TARGETS basic_node DESTINATION ./bin)
