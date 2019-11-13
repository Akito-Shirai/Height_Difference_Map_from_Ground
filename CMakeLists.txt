cmake_minimum_required(VERSION 2.8.3)
project(dhmap_from_ground)

## Compile as C++11, supported in ROS Kinetic and newer
##add_compile_options(-std=c++11)
##set(CMAKE_CXX_STANDARD 14)

#set(CMAKE_CXX_FLAGS "-march=native -O2 -std=c++11 -fopenmp")
#set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS   "-march=native -O2 -fopenmp")
#set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "-march=native -O2 -fopenmp")
#set(CMAKE_EXE_LINKER_FLAGS "-march=native -O2 -fopenmp")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  filters
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  tf
  sensor_msgs
)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenMP)
if (OPENMP_FOUND) 
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(dhmap_from_ground src/dhmap_from_ground.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(dhmap_from_ground
 ${catkin_LIBRARIES}
 ${PCL_LIBRARIES}
)
