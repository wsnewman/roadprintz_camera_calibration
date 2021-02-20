//debug_publish_image_from_pointcloud.cpp
//wsn, 10/13/20
//this is a test node for debugging/tuning
//it prompts for a filename of a png file (presumably image_from_pointcloud.png)
// and it publishes this image on the topic "/image_from_lidar"


#include <iostream>
#include <ros/ros.h>


#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//issue here: to what topic should this publish?
//HMI expects "/camera/image_rect_color" (actually, compressed version)
//but want to pipe this to a perspective transform first
//string image_topic="/camera/image_rect_color";
std::string image_topic0="/camera_pretransformed/image_raw";
std::string image_topic1="/camera_pretransformed/image_rect_color";
std::string g_image_topic;
std::string image_topic2="/virtual_camera/image_rect_color";
std::string image_topic3="/camera/image_rect_color";
std::string image_topic4="/camera/color/image_raw";

using namespace std;


cv::Mat g_image; 
sensor_msgs::ImagePtr image_msg_ptr;
double  g_image_width, g_image_height;

int g_ans;

    

int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_publish_image_from_camera"); //node name
    ros::NodeHandle nh;
    int topic_choice=0;
    image_transport::ImageTransport it(nh);
    //g_image.create(g_image_height, g_image_width, CV_8U);
    // cout<<"enter image file name (in current directory), e.g. image_from_pointcloud.png: ";
    //string fname = "image_from_pointcloud.png";
    //cin>>fname;
    string ros_ws_path = getenv("ROS_WORKSPACE");
    //ROS_INFO_STREAM("ROS_WORKSPACE: " << g_ros_ws_path << endl);
    ROS_INFO("run this node from the directory that contains the image of interest");
    cout<<"enter 0 to publish to /camera_pretransformed/image_raw"<<endl;
    cout<<"enter 1 to publish to /camera_pretransformed/image_rect_color"<<endl;
    cout<<"enter 2 to publish to /virtual_camera/image_rect_color:  "<<endl;
    cout<<"enter 3 to publish to /camera/image_rect_color:  "<<endl;
    cout<<"enter 4 to publish to /camera/color/image_raw (L515 topic):  ";

    cin>>topic_choice;
    if (topic_choice==0) {
        g_image_topic = image_topic0;
    }    
    else if (topic_choice==1) {
        g_image_topic = image_topic1;
    }
    else if (topic_choice==2) {
        g_image_topic = image_topic2;
    }
    else if (topic_choice==3) {
        g_image_topic = image_topic3; 
    }    
    else if (topic_choice==4) {
        g_image_topic = image_topic4; 
    }     
    else {
        ROS_ERROR("topic choice not recognized");
        exit(0);
    }
    image_transport::Publisher image_pub = it.advertise(g_image_topic.c_str(), 1);
    
    cout<<"enter image file name: ";
    string path_to_image;
    cin>> path_to_image;
    
    //vs hard-coded path to image
    //string path_to_image = ros_ws_path + "/src/roadprintz_ros_code/pcl_utils/image_from_camera.png";
    //ROS_INFO_STREAM("this node will attempt to republish an image from: "<<path_to_image<<endl);
    
    g_image_width = 2688;
    g_image_height = 1520;    
    
    //g_image.create(g_image_height, g_image_width, CV_8U);
    
    //read file from disk
    g_image = cv::imread(path_to_image,cv::IMREAD_COLOR); //2nd arg IMREAD_GRAYSCALE ?
    if(g_image.empty())
    {
        std::cout << "Could not read the image: " << path_to_image << std::endl;
        return 1;
    }
    
    image_msg_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", g_image).toImageMsg();  
            

    ROS_INFO_STREAM("image will be published on topic: "<<g_image_topic<<endl); ///camera/image_rect_color");

    while (ros::ok()) {
        ros::Duration(0.5).sleep();
        image_pub.publish(image_msg_ptr);
       // ROS_INFO("publishing...");
    }

    return 0;
}
