cmake_minimum_required(VERSION 2.8.3)
project(spiri_go)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib_msgs
  actionlib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


find_package(PythonLibs 2.7 REQUIRED)
find_package(PythonInterp 2.7 REQUIRED)
EXEC_PROGRAM ("${PYTHON_EXECUTABLE}"
  ARGS "-c 'import numpy; print numpy.get_include()'"
  OUTPUT_VARIABLE NUMPY_INCLUDE_DIR
  RETURN_VALUE NUMPY_NOT_FOUND)
  

# find_package(OpenCV REQUIRED)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(/usr/include/python2.7/)


###############################################
## Things needed to add messages should be here
###############################################
add_service_files(
  FILES LocalPosition.srv
)
add_action_files(
  DIRECTORY action
  FILES Takeoff.action
)
##############################
## Now generate those messages
##############################
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
)
######################
## Call catkin_package
######################
catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

#######################################
## Use Cmake commands to build the code
#######################################
add_library(spiri_data_classes src/spiri_data_classes.cpp)
target_link_libraries(spiri_data_classes ${catkin_LIBRARIES})
add_executable(spiri_go src/spiri_go.cpp)
add_dependencies(spiri_go spiri_go_generate_messages_cpp)
target_link_libraries(spiri_go spiri_data_classes ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#add_library(spiri_boost src/spiri_boost.cpp)
#target_link_libraries(spiri_boost ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} spiri_go spiri_data_classes)

# Test
if(CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)
	add_rostest_gtest(spiri_go_test test/sitl.test src/test/spiri_go_test.cpp)
	target_link_libraries(spiri_go_test ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
	endif()