cmake_minimum_required(VERSION 3.0.2)
project(legged_mujoco)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-std=c++17)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
# Include directories
# Find OpenGL package
find_package(OpenGL REQUIRED)

# Define GLFW and GLEW paths (adjust to your actual installation paths)
set(MUJOCO_DIR "$ENV{MUJOCO_PATH}")
set(GLFW_ROOT /usr/local)  # Change this to your GLFW installation path
set(GLEW_ROOT /usr)  # Change this if GLEW is installed elsewhere

# Include GLFW and GLEW directories
include_directories(${GLFW_ROOT}/include ${GLEW_ROOT}/include/GL)

# Link GLFW and GLEW libraries
find_library(GLFW_LIB glfw PATHS ${GLFW_ROOT}/lib)
if(NOT GLFW_LIB)
    message(FATAL_ERROR "GLFW library not found")
endif()

find_library(GLEW_LIB GLEW PATHS ${GLEW_ROOT}/lib/x86_64-linux-gnu)
if(NOT GLEW_LIB)
    message(FATAL_ERROR "GLEW library not found")
endif()
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED 
  COMPONENTS
  controller_manager
  hardware_interface
  roscpp
  rospy
  std_msgs
  transmission_interface
  urdf
  legged_common
)

catkin_package(
  INCLUDE_DIRS 
  include
  LIBRARIES 
  ${PROJECT_NAME}
  CATKIN_DEPENDS 
  controller_manager 
  hardware_interface  
  roscpp
  rospy 
  std_msgs 
  transmission_interface 
  urdf
  legged_common
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${MUJOCO_DIR}/include
)

link_directories(${MUJOCO_DIR}/lib)

add_library(${PROJECT_NAME}
  src/LeggedHWMujoco.cpp
)

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  mujoco
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )