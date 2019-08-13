# ----------------------------------------------------------------------------
#   Basic Configuration
# ----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.0)

project(aruco VERSION "3.1.2" LANGUAGES CXX)

set(PROJECT_SOVERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS ON) #...with compiler extensions like gnu++11


#Packages
find_package(catkin REQUIRED COMPONENTS	cv_bridge roscpp std_msgs message_generation image_transport)
find_package(OpenCV REQUIRED)

include(cmake/options.cmake)
include(cmake/findDependencies.cmake)
include(cmake/compilerOptions.cmake)

add_subdirectory(src)

if(GL_FOUND AND BUILD_GLSAMPLES)
  add_subdirectory(utils_gl)
endif()
if(BUILD_TESTS)
  add_subdirectory(tests)
endif()

include(cmake/printInfo.cmake)
include(cmake/installOptions.cmake)

# Package Generator  #######################################################
IF(BUILD_DEBPACKAGE)
include(cmake/cpack.cmake)
ENDIF()


if(CMAKE_COMPILER_IS_GNUCXX OR MINGW OR ${CMAKE_CXX_COMPILER_ID} STREQUAL Clang)
SET(THREADLIB "pthread")
ENDIF()

#Catkin dependencies
catkin_package(CATKIN_DEPENDS message_runtime roscpp std_msgs)

#Include directories
include_directories(src ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

#Aruco ROS node
add_executable(aruco_ros ros/aruco_node.cpp)
target_link_libraries(aruco_ros aruco ${catkin_LIBRARIES} ${OpenCV_LIBS} ${THREADLIB})

#Include directories
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

#Install
install(TARGETS aruco_ros
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
