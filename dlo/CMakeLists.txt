#############################################################
#                                                           #
# Copyright (c) 2022, University of California, Los Angeles #
#                                                           #
# Authors: Kenny J. Chen, Brett T. Lopez                    #
# Contact: kennyjchen@ucla.edu, btlopez@ucla.edu            #
#                                                           #
#############################################################

cmake_minimum_required(VERSION 3.12.4)
project(direct_lidar_odometry)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Release")

find_package( PCL REQUIRED )
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})

include(FindOpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else(OPENMP_FOUND)
  message("ERROR: OpenMP could not be found.")
endif(OPENMP_FOUND)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(dlo_msgs REQUIRED)

include_directories(include)
include_directories(SYSTEM)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
file(WRITE ${CMAKE_BINARY_DIR}/test_cpuid.cpp "#include <cpuid.h>")
try_compile(HAS_CPUID ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
file(REMOVE ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
if(HAS_CPUID)
  add_compile_definitions(HAS_CPUID)
endif()

# NanoFLANN
add_library(nanoflann STATIC
  src/nano_gicp/nanoflann.cc
)
target_link_libraries(nanoflann ${PCL_LIBRARIES})
target_include_directories(nanoflann PUBLIC include ${PCL_INCLUDE_DIRS})
ament_target_dependencies(nanoflann rclcpp std_msgs sensor_msgs geometry_msgs pcl_ros nav_msgs)

# NanoGICP
add_library(nano_gicp STATIC
  src/nano_gicp/lsq_registration.cc
  src/nano_gicp/nano_gicp.cc
)
target_link_libraries(nano_gicp ${PCL_LIBRARIES} ${OpenMP_LIBS} nanoflann)
target_include_directories(nano_gicp PUBLIC include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(nano_gicp rclcpp std_msgs sensor_msgs geometry_msgs pcl_ros nav_msgs)

## Odometry Node
add_executable(dlo_odom_node src/dlo/odom_node.cc src/dlo/odom.cc)
target_compile_options(dlo_odom_node PRIVATE ${OpenMP_FLAGS})
target_link_libraries(dlo_odom_node ${PCL_LIBRARIES} ${OpenMP_LIBS} Threads::Threads nano_gicp)
ament_target_dependencies(dlo_odom_node rclcpp std_msgs sensor_msgs geometry_msgs pcl_ros nav_msgs dlo_msgs)

## Mapping Node
add_executable (dlo_map_node src/dlo/map_node.cc src/dlo/map.cc)
target_compile_options(dlo_map_node PRIVATE ${OpenMP_FLAGS})
target_link_libraries(dlo_map_node ${PCL_LIBRARIES} ${OpenMP_LIBS} Threads::Threads)
ament_target_dependencies(dlo_map_node rclcpp std_msgs sensor_msgs geometry_msgs pcl_ros nav_msgs dlo_msgs)

install (TARGETS dlo_odom_node dlo_map_node
  DESTINATION lib/${PROJECT_NAME}
)

install (DIRECTORY cfg launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} nano_gicp nanoflann)
ament_export_dependencies(rclcpp std_msgs sensor_msgs geometry_msgs pcl_ros nav_msgs dlo_msgs)
ament_package()
