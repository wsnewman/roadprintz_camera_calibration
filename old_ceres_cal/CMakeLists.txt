cmake_minimum_required(VERSION 2.8.3)
project(roadprintz_camera_calibration)

find_package(catkin_simple REQUIRED)

#uncomment next line to use OpenCV library
find_package(OpenCV REQUIRED)

#uncomment the next 2 lines to use the point-cloud library
#find_package(PCL 1.7 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})


#uncomment the following 4 lines to use the Eigen library
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

#uncomment the following 4 lines to use the Eigen library
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

# Libraries: uncomment the following and edit arguments to create a new library
cs_add_library(circle_detector src/circle_detector.cpp)   
cs_add_library(openCV_utils src/openCV_utils.cpp)  

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
#cs_add_executable(find_red_pixels src/find_red_pixels.cpp)
#cs_add_executable(find_features src/find_features.cpp)
#cs_add_executable(draw_polylines src/draw_polylines.cpp)
cs_add_executable(interactive_image_gui src/interactive_image_gui.cpp)
cs_add_executable(interactive_image_gui2 src/interactive_image_gui2.cpp)
cs_add_executable(interactive_image_gui3 src/interactive_image_gui3.cpp)

cs_add_executable(perspective_transform src/perspective_transform.cpp)
cs_add_executable(perspective_transform_v2 src/perspective_transform_v2.cpp)


cs_add_executable(ipcam src/ipcam.cpp)
cs_add_executable(ipcam_svc src/ipcam_svc.cpp)
cs_add_executable(test_find_contours src/test_find_contours.cpp)
cs_add_executable(test_find_patterns src/test_find_patterns.cpp)
cs_add_executable(extrinsic_calib src/extrinsic_calib.cpp)
cs_add_executable(rp_find_template src/rp_find_template.cpp)
cs_add_executable(analyze_image17 src/analyze_image17.cpp)
cs_add_executable(analyze_image16 src/analyze_image16.cpp)
cs_add_executable(find_poster src/find_poster.cpp)
cs_add_executable(find_poster_from_lidar src/find_poster_from_lidar.cpp)
cs_add_executable(find_extrinsics src/find_extrinsics.cpp)
cs_add_executable(find_extrinsics_v2 src/find_extrinsics_v2.cpp)

cs_add_executable(find_poster_9x7 src/find_poster_9x7.cpp)

cs_add_executable(fake_publish_camera_from_image src/fake_publish_camera_from_image.cpp)
cs_add_executable(openCV_utils_example_main src/openCV_utils_example_main.cpp)

cs_add_executable(cam_poster_calib_test_service_node src/cam_poster_calib_test_service_node.cpp)
cs_add_executable(cam_poster_calib_test_main src/cam_poster_calib_test_main.cpp)


#cs_add_executable(wsn_ceres_helloworld src/wsn_ceres_helloworld.cpp)

#cs_add_executable(harris_corners src/harris_corners.cpp)
#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
# target_link_libraries(example my_lib)
#target_link_libraries( find_red_pixels ${OpenCV_LIBS})
#target_link_libraries( find_features ${OpenCV_LIBS})
#target_link_libraries( draw_polylines ${OpenCV_LIBS})
target_link_libraries( interactive_image_gui ${OpenCV_LIBS})
target_link_libraries( interactive_image_gui2 ${OpenCV_LIBS})
target_link_libraries( interactive_image_gui3 ${OpenCV_LIBS})

target_link_libraries( perspective_transform ${OpenCV_LIBS})
target_link_libraries( perspective_transform_v2 ${OpenCV_LIBS})


target_link_libraries( ipcam ${OpenCV_LIBS})
target_link_libraries( ipcam_svc ${OpenCV_LIBS})
target_link_libraries( test_find_contours circle_detector ${OpenCV_LIBS})
target_link_libraries( test_find_patterns circle_detector ${OpenCV_LIBS})
target_link_libraries( extrinsic_calib  ${OpenCV_LIBS})
target_link_libraries( rp_find_template  ${OpenCV_LIBS})
target_link_libraries( analyze_image17  ${OpenCV_LIBS})
target_link_libraries( analyze_image16  ${OpenCV_LIBS})
target_link_libraries( find_poster  ${OpenCV_LIBS})
target_link_libraries( find_poster_9x7  ${OpenCV_LIBS})
target_link_libraries( find_poster_from_lidar  ${OpenCV_LIBS})
target_link_libraries( find_extrinsics  ${OpenCV_LIBS})
target_link_libraries( find_extrinsics_v2  ${OpenCV_LIBS})

target_link_libraries(fake_publish_camera_from_image  ${OpenCV_LIBS})

target_link_libraries( openCV_utils_example_main openCV_utils ${OpenCV_LIBS})
target_link_libraries( cam_poster_calib_test_service_node openCV_utils ${OpenCV_LIBS})


cs_install()
cs_export()
     
