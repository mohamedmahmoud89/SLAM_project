cmake_minimum_required (VERSION 2.8.3)
project (SLAM)
SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-g -std=c++14 -pthread")
include_directories(
	../src/cfg
	../src/supp/file_mgr
	../src/supp/raii
	../src/supp/common
	../src/supp/matrix
	../src/supp/gaus
	../src/fuslib/motion
	../src/fuslib/scan
	../src/fuslib/feat
	../src/odom
	../src/lfx	
)
add_executable(SLAM 
	../src/cfg/robot_cfg.cpp
	../src/supp/file_mgr/file_mgr.cpp 
	../src/supp/matrix/matrix.cpp
	../src/supp/gaus/gaus.cpp
	../src/main/main.cpp
	../src/odom/odom.cpp
	../src/lfx/lfx.cpp
	../src/fuslib/feat/feat.cpp
)
