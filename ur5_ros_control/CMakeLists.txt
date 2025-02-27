cmake_minimum_required(VERSION 2.8.3)
project(ur5_ros_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  roscpp
  rospy
  std_msgs
  std_srvs
  message_generation
  controller_interface
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
  INCLUDE_DIRS include
#  LIBRARIES ros_control_example
  CATKIN_DEPENDS controller_manager roscpp rospy std_msgs std_srvs controller_interface message_generation
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_executable(ur5_ros_control_sim src/ur5_ros_control.cpp) 
target_link_libraries(ur5_ros_control_sim
  ${catkin_LIBRARIES} 
)

# add_executable(ur5_ros_control_real src/ur5_ros_control_real.cpp) 
# target_link_libraries(ur5_ros_control_real
#   ${catkin_LIBRARIES} 
# )

## Declare a cpp library
add_library(${PROJECT_NAME}
        src/ur5_controllers.cpp
        )

target_link_libraries(${PROJECT_NAME}
${catkin_LIBRARIES}
)

target_compile_options(${PROJECT_NAME} PUBLIC)

# Install plugins
install(FILES legged_mpc_controllers_plugins.xml
        #       arm_mpc_controllers_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )