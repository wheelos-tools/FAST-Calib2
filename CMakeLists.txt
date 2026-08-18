cmake_minimum_required(VERSION 3.10)
project(fast_calib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

find_package(PCL REQUIRED COMPONENTS common io filters features segmentation
                                     search kdtree sample_consensus registration)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  src
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})
# Common::Point is a custom point type: build the PCL templates header-only so
# every instantiation exists regardless of header include order.
add_definitions(-DPCL_NO_PRECOMPILE)
link_directories(${PCL_LIBRARY_DIRS})

add_executable(fast_calib src/main.cpp)
target_link_libraries(fast_calib ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(lidar_center_test src/lidar_center_test.cpp)
target_link_libraries(lidar_center_test ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(multi_fast_calib src/multi_scene.cpp)
target_link_libraries(multi_fast_calib ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
