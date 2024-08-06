cmake_minimum_required(VERSION 3.0.2)
project(legged_unitree_mujoco)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED 
  COMPONENTS
  controller_manager
  hardware_interface
  roscpp
  std_msgs
  legged_mujoco
)

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
  legged_common
  legged_mujoco
  DEPENDS 
)

set(MUJOCO_DIR "$ENV{MUJOCO_PATH}")
set(GLFW_DIR "/usr")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${MUJOCO_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${GLFW_DIR}/include
)

link_directories(${MUJOCO_DIR}/lib)
link_directories(${GLFW_DIR}/lib/x86_64-linux-gnu)

## Declare cpp executables
add_executable(${PROJECT_NAME}
  src/legged_unitree_mujoco.cpp
)

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  mujoco
  glfw
  GL
  pthread
)

add_dependencies(legged_unitree_mujoco
  ${catkin_EXPORTED_TARGETS}
)

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

# Mark other files for installation
install(DIRECTORY config launch lib
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
# cmake_minimum_required(VERSION 3.0.2)
# project(legged_unitree_mujoco)

# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# ## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++17)

# ## Find catkin macros and libraries
# ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# ## is used, also find other catkin packages
# find_package(catkin REQUIRED 
#   COMPONENTS
#   controller_manager
#   hardware_interface
#   roscpp
#   std_msgs
#   legged_mujoco
# )

# ###################################
# ## catkin specific configuration ##
# ###################################
# ## The catkin_package macro generates cmake config files for your package
# ## Declare things to be passed to dependent projects
# ## INCLUDE_DIRS: uncomment this if your package contains header files
# ## LIBRARIES: libraries you create in this project that dependent projects also need
# ## CATKIN_DEPENDS: catkin_packages dependent projects also need
# ## DEPENDS: system dependencies of this project that dependent projects also need
# catkin_package(
#   INCLUDE_DIRS 
#   include
#   LIBRARIES 
#   ${PROJECT_NAME}
#   CATKIN_DEPENDS 
#   controller_manager 
#   hardware_interface 
#   mujoco 
#   roscpp
#   rospy 
#   std_msgs 
#   legged_common
#   legged_mujoco
#   DEPENDS 
# )

# set(MUJOCO_DIR "$ENV{HOME}/.mujoco/mujoco-3.1.6")
# set(GLFW_DIR "/usr")

# ###########
# ## Build ##
# ###########

# ## Specify additional locations of header files
# ## Your package locations should be listed before other locations
# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   ${MUJOCO_DIR}/include
#   ${GLFW_DIR}/include
#   ${legged_mujoco_INCLUDE_DIRS}
# )

# link_directories(${MUJOCO_DIR}/lib)
# link_directories(${GLFW_DIR}/lib/x86_64-linux-gnu)

# ## Declare cpp executables
# add_executable(${PROJECT_NAME}
#   src/legged_unitree_mujoco.cpp
# )

# ## Specify libraries to link executable targets against
# target_link_libraries(${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   mujoco
#   glfw
#   GL
#   pthread
# )

# add_dependencies(legged_unitree_mujoco
#   ${catkin_EXPORTED_TARGETS}
# )

# # Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME}
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#         )

# # Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#         FILES_MATCHING PATTERN "*.h"
#         )

# # Mark other files for installation
# install(DIRECTORY config launch lib
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#         )