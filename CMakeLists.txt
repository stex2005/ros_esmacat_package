cmake_minimum_required(VERSION 2.8.4)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules")

#if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
# Default to installing in SOEM source directory
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install)
#endif()

project(esmacat_pkg C CXX)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
    )

## System dependencies are found with CMake's conventions
## Boost thread needed to handle mutex
find_package(Boost REQUIRED COMPONENTS
    thread
    )


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
    DIRECTORY
    msg
    FILES
    esmacat_sensor.msg
    esmacat_command.msg
    )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
    )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES esmacat_pkg
    CATKIN_DEPENDS roscpp message_runtime
    # DEPENDS system_lib
    )

## TODO Not used
set(SOEM_INCLUDE_INSTALL_DIR include/ethercat_driver)
set(ESMACAT_INCLUDE_INSTALL_DIR include/esmacat_library)

###################################
##   OS specific configuration   ##
###################################

if(WIN32)
    set(OS "win32")
    include_directories(${SOEM_INCLUDE_INSTALL_DIR}/oshw/win32/wpcap/Include)
    link_directories(${CMAKE_SOURCE_DIR}/${SOEM_INCLUDE_INSTALL_DIR}/oshw/win32/wpcap/Lib/x64)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /D _CRT_SECURE_NO_WARNINGS")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  /W4")
    set(OS_LIBS wpcap.lib Packet.lib Ws2_32.lib Winmm.lib)
elseif(UNIX)
    set(OS "linux")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
    set(OS_LIBS pthread rt)
elseif(${CMAKE_SYSTEM_NAME} MATCHES "rt-kernel")
    set(OS "rtk")
    message("ARCH is ${ARCH}")
    message("BSP is ${BSP}")
    include_directories(${SOEM_INCLUDE_INSTALL_DIR}/oshw/${OS}/${ARCH})
    file(GLOB OSHW_EXTRA_SOURCES ${SOEM_INCLUDE_INSTALL_DIR}/oshw/${OS}/${ARCH}/*.c)
    set(OSHW_SOURCES "${OS_HW_SOURCES} ${OSHW_ARCHSOURCES}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-but-set-variable")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-function")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-format")
    set(OS_LIBS "-Wl,--start-group -l${BSP} -l${ARCH} -lkern -ldev -lsio -lblock -lfs -lusb -llwip -leth -li2c -lrtc -lcan -lnand -lspi -lnor -lpwm -ladc -ltrace -lc -lm -Wl,--end-group")
endif()

message("OS is ${OS}")
message("SYSTEM is ${CMAKE_SYSTEM_NAME}")

################
# Global Files #
################

file(GLOB SOEM_SOURCES include/ethercat_driver/soem/*.c)
file(GLOB UTIL_SOURCES include/ethercat_driver/utilities/*.c)
file(GLOB OSAL_SOURCES include/ethercat_driver/osal/${OS}/*.c)
file(GLOB OSHW_SOURCES include/ethercat_driver/oshw/${OS}/*.c)
file(GLOB ESMACAT_SOURCES
    include/esmacat_library/*.cpp
    include/esmacat_slave_drivers/drivers/*.cpp
    #include/esmacat_applications/drivers/*.cpp
    )

file(GLOB SOEM_HEADERS include/ethercat_driver/soem/*.h)
file(GLOB UTIL_HEADERS include/ethercat_driver/utilities/*.h)
file(GLOB OSAL_HEADERS include/ethercat_driver/osal/osal.h osal/${OS}/*.h)
file(GLOB OSHW_HEADERS include/ethercat_driver/oshw/${OS}/*.h)
file(GLOB ESMACAT_HEADERS
    include/esmacat_library/*.h
    include/esmacat_slave_drivers/drivers/*.h
    #include/esmacat_applications/drivers/*.h
    )

#############
## Include ##
#############

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(

    # include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    include/${PROJECT_NAME}
#    include/epos4_master/epos4_driver
    include/ethercat_driver/soem
    include/ethercat_driver/osal
    include/ethercat_driver/osal/${OS}
    include/ethercat_driver/oshw/${OS}
    include/esmacat_library
    include/esmacat_slave_drivers/drivers
    )

#############
## Library ##
#############

## Declare C++ libraries

## SOEM Library
add_library(ethercat_driver STATIC
    ${UTIL_SOURCES}
    ${SOEM_SOURCES}
    ${OSAL_SOURCES}
    ${OSHW_SOURCES}
    ${OSHW_EXTRA_SOURCES}
    )
target_link_libraries(ethercat_driver ${OS_LIBS} )

## EsmaCAT Software Library
add_library(esmacat STATIC
    ${ESMACAT_SOURCES}
    )
target_link_libraries(esmacat
    ethercat_driver
    )


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
#add_executable(basic_node src/basic_node.cpp)
#add_executable(talker_node src/talker.cpp)
#add_executable(INTERFACE_node src/INTERFACE.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#target_link_libraries(basic_node
#    ${catkin_LIBRARIES}
#    )

#target_link_libraries(talker_node
#    ${catkin_LIBRARIES})

#target_link_libraries(INTERFACE_node
#    ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
#install(TARGETS basic_node
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#    )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_esmacat_pkg.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

## Add nodes CMake directories
add_subdirectory(src/epos4_master)
add_subdirectory(src/smartbox_interface)
