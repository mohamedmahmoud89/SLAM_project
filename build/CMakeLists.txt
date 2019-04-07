cmake_minimum_required (VERSION 2.8.3)
project (SLAM)
SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-g -std=c++14 -pthread")
include_directories(
	../src/supp/file_mgr
	../src/supp/raii
	../src/supp/common
	../src/odom
	../src/lfx	
)
add_executable(SLAM 
	../src/supp/file_mgr/file_mgr.cpp 
	../src/main/main.cpp
	../src/odom/odom.cpp
	../src/lfx/lfx.cpp
)
