cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#Important CMAKE stuff
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

#include directories
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR})

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

#Eigen
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# Build the plugins
add_library(panda_control_plugin SHARED src/panda_control_plugin.cpp)
target_link_libraries(panda_control_plugin ${GAZEBO_LIBRARIES} Eigen3::Eigen)

#Build the executable
add_executable(csc376-assignment main.cpp include/gazebocontroller.h src/gazebocontroller.cpp include/pandarobotmodel.h src/pandarobotmodel.cpp)
target_link_libraries(csc376-assignment ${GAZEBO_LIBRARIES} Eigen3::Eigen)

