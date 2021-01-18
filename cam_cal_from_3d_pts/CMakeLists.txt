cmake_minimum_required(VERSION 2.8.3)
project(cam_cal_from_3d_pts)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cc_utils
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES cam_cal_3d
)

add_library(cam_cal_3d src/3dlib.cpp)  
target_link_libraries(cam_cal_3d 
	${OpenCV_LIBS}
	${CERES_LIBRARIES}
	${catkin_LIBRARIES}
)

add_executable(test_main_interp src/test_main_interp.cpp)
target_link_libraries(test_main_interp
	${OpenCV_LIBS}
	${catkin_LIBRARIES}
	yaml-cpp
	
	cam_cal_3d
)

add_executable(test_main_four src/test_main_four.cpp)
target_link_libraries(test_main_four
	${OpenCV_LIBS}
	${catkin_LIBRARIES}
	yaml-cpp
	
	cam_cal_3d
)
