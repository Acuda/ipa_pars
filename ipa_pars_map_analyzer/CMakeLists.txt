cmake_minimum_required(VERSION 2.8.3)
project(ipa_pars_map_analyzer)

set(catkin_RUN_PACKAGES
	roscpp
	roslib
	actionlib
	actionlib_msgs
	std_msgs
	sensor_msgs
	geometry_msgs
	cv_bridge
)

set(catkin_BUILD_PACKAGES
	${catkin_RUN_PACKAGES}
	cmake_modules
	message_generation
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS ${catkin_BUILD_PACKAGES} genmsg actionlib_msgs)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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
add_action_files(
   DIRECTORY action
   FILES ParsMapAnalyzer.action ParsMapTesselation.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   actionlib_msgs 
   geometry_msgs
   sensor_msgs
   std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
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
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS 
	common/include
	ros/include
  LIBRARIES #ipa_pars_map_analyzer_server
  CATKIN_DEPENDS ${catkin_RUN_PACKAGES} actionlib_msgs message_runtime
  DEPENDS
	OpenCV
	Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	common/include
	ros/include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

## map analyzer main server
add_executable(ipa_pars_map_analyzer_server
	ros/src/ipa_pars_map_analyzer_server.cpp)

target_link_libraries(ipa_pars_map_analyzer_server
	${catkin_LIBRARIES} 
	${OpenCV_LIBRARIES})
add_dependencies(ipa_pars_map_analyzer_server
	${catkin_EXPORTED_TARGETS}
	${${PROJECT_NAME}_EXPORTED_TARGETS})

## map tesselation server
add_executable(ipa_pars_map_tesselation_server
	ros/src/ipa_pars_map_tesselation_server.cpp)

target_link_libraries(ipa_pars_map_tesselation_server
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES})
add_dependencies(ipa_pars_map_tesselation_server
	${catkin_EXPORTED_TARGETS}
	${${PROJECT_NAME}_EXPORTED_TARGETS})

### client for testing purpose
add_executable(ipa_pars_map_analyzer_client ros/src/ipa_pars_map_analyzer_client.cpp)
target_link_libraries(ipa_pars_map_analyzer_client
	${catkin_LIBRARIES} 
	${OpenCV_LIBRARIES})
add_dependencies(ipa_pars_map_analyzer_client
	${catkin_EXPORTED_TARGETS}
	${${PROJECT_NAME}_EXPORTED_TARGETS})


#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS ipa_pars_map_analyzer_client #ipa_pars_map_analyzer_server
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

##uncomment this if you have a common-directory with header-files
install(DIRECTORY common/include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)

##uncomment this if you have header-files in your project
install(DIRECTORY ros/include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)

#install(DIRECTORY scripts
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#	PATTERN ".svn" EXCLUDE
#)

#install(DIRECTORY ros/launch
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros
#	PATTERN ".svn" EXCLUDE
#)

#install(DIRECTORY common/files
#	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common
#	PATTERN ".svn" EXCLUDE
#)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ipa_pars_map_analyzer.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
