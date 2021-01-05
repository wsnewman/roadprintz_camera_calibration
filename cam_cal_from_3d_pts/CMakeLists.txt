cmake_minimum_required(VERSION 2.8.3)
project(cam_cal_from_3d_pts)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES cam_cal_3d
)

add_library(cam_cal_3d src/3dlib.cpp)  
target_link_libraries(cam_cal_3d 
	yaml-cpp
	${OpenCV_LIBS}
	${catkin_LIBRARIES}
)

add_executable(test_main src/test_main.cpp)
target_link_libraries(test_main
	${OpenCV_LIBS}
	${catkin_LIBRARIES}
	
	cam_cal_3d
)
