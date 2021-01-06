// sample code that grabbed images from ipcam
// convert this to a service: take and store a snapshot for each trigger;
// increment the name of the image for each saved image

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/SetBool.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <string.h>

#include <iostream>
#include <string>

static const std::string OPENCV_WINDOW = "Open-CV display window";

#include <iostream>
//#include <thread>
//#include "opencv2/opencv.hpp"
#include <vector>
using namespace std;
using namespace cv;
string g_path_to_images = "";
string g_image_name = "image";
int g_imagenum = 0;

// true means overwrite
bool snap_callback(std_srvs::SetBoolRequest &request, std_srvs::SetBoolResponse &response)
{
    ROS_INFO("snapshot callback activated");
    VideoCapture cap("http://admin:R0adPrintz@192.168.1.190/cgi-bin/snapshot.cgi?");
    Mat ip_image;
    cap >> ip_image;
    ROS_INFO("%d", cap.isOpened());
    response.success = cap.isOpened();
    if (!response.success)
    {
        return true;
    }
    string image_name;
    if (request.data)
    {
        image_name = g_path_to_images + "image.jpg";
    }
    else
    {
        time_t now;
        time(&now);
        char buf[sizeof "0000-00-00T00:00:00"];
        strftime(buf, sizeof buf, "%FT%T", gmtime(&now));
        image_name = g_path_to_images + buf + ".jpg";
    }
    response.message = image_name;
    imwrite(image_name, ip_image);
    cout << "saved image to " << image_name << endl;

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ipcam");
    ros::NodeHandle n; //
    string ros_ws_path = getenv("ROS_WORKSPACE");
    g_path_to_images = ros_ws_path + "/src/roadprintz_ros_code/example_opencv/images/";
    // cout << "enter red ratio threshold: (e.g. 10) ";
    // cin >> g_redratio;
    ros::Duration timer(1);
    // cv::namedWindow("Display window", WINDOW_AUTOSIZE); // Create a window for display.

    ros::ServiceServer service = n.advertiseService("ipcam_snapshot", snap_callback);

    ros::spin();
    return 0;
}
