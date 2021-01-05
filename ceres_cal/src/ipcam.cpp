//sample code that grabbed images from ipcam


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Open-CV display window";

#include <iostream>
//#include <thread>
//#include "opencv2/opencv.hpp"
#include <vector>
using namespace std;
using namespace cv;


//cv::namedWindow(OPENCV_WINDOW);

int main(int argc, char** argv) {
    ros::init(argc, argv, "ipcam");
    ros::NodeHandle n; //        
    //cout << "enter red ratio threshold: (e.g. 10) ";
    //cin >> g_redratio;
    ros::Duration timer(1);
    //cv::namedWindow("Display window", WINDOW_AUTOSIZE); // Create a window for display.

    while (ros::ok()) {
        VideoCapture cap("http://admin:R0adPrintz@192.168.0.190/cgi-bin/snapshot.cgi?");
        if (cap.isOpened()) {
            ROS_INFO("opened!");
            Mat frame;
            cap >> frame;
            // Update GUI Window; this will display processed images on the open-cv viewer.
            cv::imshow(OPENCV_WINDOW, frame);
            cv::waitKey(1); //need waitKey call to update OpenCV image window            
        } else {
            ROS_WARN("not opened");
        }

        ros::spinOnce();
        timer.sleep();
    }
    return 0;
}


