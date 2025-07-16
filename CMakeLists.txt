cmake_minimum_required(VERSION 3.0.2)
project(prueba1_linea)

## Find catkin macros and libraries
## if COMPONENTS list includes both catkin and non-catkin packages, catkin will prefer the catkin versions if they exist
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  trajectory_msgs
  std_msgs
)

## Uncomment this if the package has a setup.py. This macro enables
## you to install Python scripts and libraries into an installation
## space.
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin package in cmake recreates the cmake common modules and search paths
## using catkin specific configuration (`/prueba1_linea-extras.cmake`) and sets up
## the appropriate environment for building, testing and installing.
## For more details see http://ros.org/doc/api/catkin/html/user_guide/html
# catkin_package()
catkin_package(
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    trajectory_msgs
    moveit_ros_planning_interface
    moveit_visual_tools
    std_msgs
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

## Declare a C++ executable
## With catkin_make all of these will be in a single combined package.
## Add launch configuration for new nodes, like this:
## install(PROGRAMS scripts/my_python_node
##   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
## )

# Añade tu ejecutable C++ aquí
add_executable(lissajous_node src/lissajous_node.cpp) # <---- Asegúrate de que 'lissajous_node.cpp' es el nombre correcto de tu archivo fuente.

## Add install rules for C++ executables
install(TARGETS lissajous_node # <---- y aquí
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Declare a library that should be installed
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}.cpp
# )

## Add install rules for libraries
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )

## Add dependencies to the build system for your executable.
# Las librerías de las que depende tu ejecutable
target_link_libraries(lissajous_node # <---- y aquí
  ${catkin_LIBRARIES}
  ${MoveIt_LIBRARIES} # Esto es crucial para enlazar con MoveIt!
)
