cmake_minimum_required(VERSION 3.15)
project(inverted_pendulum_simulator)

find_package(mujoco 2.3 REQUIRED)

find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

add_compile_definitions(PROJECT_DIR="${PROJECT_SOURCE_DIR}")


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
#  INCLUDE_DIRS include
#  LIBRARIES inverted_pendulum_simulator
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  include
  ${catkin_INCLUDE_DIRS}
  ${mujoco_INCLUDE_DIRS}
)

add_library(simulator_libs STATIC
  src/mujoco_view.cpp
  src/inverted_pendulum.cpp
)
target_link_libraries(simulator_libs
  glfw
  OpenGL::GL
)
target_include_directories(simulator_libs 
    PUBLIC
    include
    ${mujoco_INCLUDE_DIRS}
)

add_executable(simulator src/simulator.cpp)
target_link_libraries(simulator ${catkin_LIBRARIES} simulator_libs mujoco::mujoco ${GLFW3_LIBRARY})