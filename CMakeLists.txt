cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pclCompressor)
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pclCompressor ./pclCompressor.cpp)
target_link_libraries (pclCompressor ${PCL_LIBRARIES})
