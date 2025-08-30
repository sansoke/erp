cmake_minimum_required(VERSION 2.8.3)
project(virtual_frame_broadcaster)

# Set to Release in order to speed up the program significantly
set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  roscpp
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
)
message(STATUS "System: ${CMAKE_SYSTEM}")
## System dependencies are found with CMake's conventions
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
message(STATUS "${CMAKE_MODULE_PATH}")

# Eigen3 FindScript Backward compatibility (ubuntu saucy)
# Since FindEigen.cmake is deprecated starting from jade.
if (EXISTS "FindEigen3.cmake")
  find_package(Eigen3 REQUIRED)
  set(Eigen_INCLUDE_DIRS ${Eigen3_INCLUDE_DIRS})
elseif (EXISTS "FindEigen.cmake")
  find_package(Eigen REQUIRED)
elseif (EXISTS "FindEigen.cmake")
  message(WARNING "No findEigen cmake script found. You must provde one of them,
  e.g. by adding it to ${PROJECT_SOURCE_DIR}/cmake_modules.")
endif (EXISTS "FindEigen3.cmake")

set(EXTERNAL_INCLUDE_DIRS ${Eigen_INCLUDE_DIRS})

IF(NOT MSVC)
  include(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support which is required 
  by linked third party packages starting from ROS Jade. Ignore this message for ROS Indigo.")
endif()
endif()

catkin_package(
  INCLUDE_DIRS include ${EXTERNAL_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} ${EXTERNAL_LIBS}
  CATKIN_DEPENDS
	geometry_msgs
	roscpp
  tf2
  tf2_ros
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  SYSTEM
  ${EXTERNAL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/virtual_frame_broadcaster.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${EXTERNAL_LIBS}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/virtual_frame_broadcaster_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${EXTERNAL_LIBS}
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   #FILES_MATCHING PATTERN "*.h"
)
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

