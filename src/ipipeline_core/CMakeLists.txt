cmake_minimum_required(VERSION 2.8.3)
project(ipipeline_core)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV REQUIRED)

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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ipipeline_core
  CATKIN_DEPENDS roscpp sensor_msgs
  DEPENDS Imlib2 pthread ${OpenCV_LIBRARIES}
)

###########
## Build ##
###########
SET(CFG_DIR ${PROJECT_SOURCE_DIR}/src/config)
SET_SOURCE_FILES_PROPERTIES(${CFG_DIR}/config.tab.cpp GENERATED)

SET (YACC_COMPILE bison)

ADD_CUSTOM_COMMAND(
	OUTPUT   ${CFG_DIR}/config.tab.cpp
	DEPENDS	  ${CFG_DIR}/config.y 
	COMMAND   ${YACC_COMPILE}
	ARGS      -v -p config -t -d -o ${CFG_DIR}/config.tab.cpp ${CFG_DIR}/config.y
)


SET_SOURCE_FILES_PROPERTIES(${CFG_DIR}/lex.config.cpp GENERATED)

SET (FLEX_COMPILE lex)

ADD_CUSTOM_COMMAND(
	OUTPUT   ${CFG_DIR}/lex.config.cpp
	DEPENDS	  ${CFG_DIR}/config.lex ${CFG_DIR}/config.tab.cpp
	COMMAND   ${FLEX_COMPILE}
	ARGS       -P config -o ${CFG_DIR}/lex.config.cpp ${CFG_DIR}/config.lex
)

ADD_DEFINITIONS(-D_ISOC99_SOURCE)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#common commands for building c++ executables and libraries
add_library(ipipeline_core
    ${CFG_DIR}/Config.cpp
    ${CFG_DIR}/config.tab.cpp
    ${CFG_DIR}/lex.config.cpp
    src/IPLBarrier.cpp
    src/IPLBinaryOperator.cpp
    src/IPLBufferWriter.cpp
    src/IPLCachingSource.cpp
    src/IPLCheckFloat.cpp
    src/IPLDelayedImage.cpp
    src/IPLDemux.cpp
    src/IPLDriver.cpp
    src/IPLIPScheduler.cpp
    src/IPLIPScheduler1T.cpp
    src/IPLIPScheduler1Tstatic.cpp
    src/IPLIPSchedulerMT.cpp
    src/IPLImageFilter.cpp
    src/IPLImageProcessor.cpp
    src/IPLImageSaver.cpp
    src/IPLExternalImageSource.cpp
    src/IPLImageSource.cpp
    src/IPLImageSourceWithView.cpp
    src/IPLCvImageSource.cpp
    src/IPLMultiImageSource.cpp
    src/IPLGenericProbe.cpp
    #src/IppiImage.cpp
    #src/IppiText.cpp # Replace with cvPutText, in modules
)

# target_link_libraries(${PROJECT_NAME} 	ippi ippcc ippcv ippsr ipps Imlib2 pthread)
target_link_libraries(ipipeline_core Imlib2 pthread ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

add_executable(ipipeline_core_test test/IPLTest.cpp test/IPLTestDriver.cpp)
target_link_libraries(ipipeline_core_test ipipeline_core)

add_executable(ipipeline_core_testapp test/testapp.cpp)
target_link_libraries(ipipeline_core_testapp ipipeline_core)

add_executable(ipipeline_core_testcfg test/testcfg.cpp)
target_link_libraries(ipipeline_core_testcfg ipipeline_core ) 


## Declare a cpp library
# add_library(ipipeline_core
#   src/${PROJECT_NAME}/ipipeline_core.cpp
# )

## Declare a cpp executable
# add_executable(ipipeline_core_node src/ipipeline_core_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(ipipeline_core_node ipipeline_core_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(ipipeline_core_node
#   ${catkin_LIBRARIES}
# )

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

## Mark executables and/or libraries for installation
install(TARGETS ipipeline_core 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ipipeline_core.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
