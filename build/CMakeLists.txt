cmake_minimum_required (VERSION 2.8.3)
project (SLAM)
SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-g -std=c++14 -pthread")
include_directories(
	../src/file_mgr
	../src/raii
	../src/common
	../src/odom	
)
add_executable(SLAM 
	../src/file_mgr/file_mgr.cpp 
	../src/main/main.cpp
	../src/odom/odom.cpp
)
