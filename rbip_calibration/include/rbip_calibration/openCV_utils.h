#ifndef OPENCV_UTILS_H_
#define OPENCV_UTILS_H_

#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d.hpp" //need this for findChessboardCorners()
#include <iostream>
#include <fstream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
using namespace cv;
using namespace std;

#include <ros/ros.h> //ALWAYS need to include this
static const std::string OPENCV_WINDOW = "ImageWindow";
Mat g_src;
Mat g_copy_of_image,g_copy_of_image_markup;


bool g_got_new_image=true; //this will supress attempt to find poster on start-up
bool g_found_poster=false;
bool g_preferred_orientation=false; 
//not thrilled about communicating values as follows via global vars...ugh
int g_n_keypts_wide=8; //default to small poster
int g_n_keypts_high=6; //default

int index_key_point = 27; 

double g_x_pix_upper_left,g_y_pix_upper_left; //to choose "upper-left" point in poster
double g_x_pix_keypt,g_y_pix_keypt; //generalize: use index_key_point to select (u,v) of interest


//default topics; can change these in application BEFORE instantiating 
std::string g_output_image_topic="/camera/image_markup";
std::string g_input_image_topic="/camera/image_rect_color";

class OpenCvUtils {
public:
    OpenCvUtils() {}; //constructor;
    OpenCvUtils(string image_topic); //constructor;
    //call next fnc to find key points of a checkerboard
    //specify how many key points: n_keypts_wide x n_keypts_high interior corners among squares
    // this is the number of intersections, not the number of squares
    // input the directory path from which to find an image file of interest;
    // results will be written to this same directory
    //input the image from disk (fname_image_root, WITHOUT .png suffix), and output the key points (u,v) in a CSV file in same directory and with same prefix name
    //this also dumps an image with overlayed keypoints markup to file: path/fname_image_root_processed.png
    //returns "true" if found expected key points, else false
    //ALSO, sets "preferred_orientation" to "true" if vision target is in preferred orientation (e.g. to match up with robot laser-pointer acquired data)
    //also sets values of x_upper_left and y_upper_left (fore/port of checkerboard)...useful for vision-target based robot motion, e.g.
    // to test camera calibration
    bool find_poster(string path, string fname_image_root,int n_keypts_wide, int n_keypts_high, bool &preferred_orientation, double &x_upper_left, double &y_upper_left);


private:
};

class ImageTransportUtil
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  //void ImageConverterMouseCB(int event, int x, int y, int flags, void* userdata);
public:
  ImageTransportUtil(); //: it_(nh_);
  /*
  ~ImageTransportUtil();
  
  {
    cv::destroyWindow(OPENCV_WINDOW);
    //cv::destroyWindow(OPENCV_ZOOM_WINDOW);
  }*/
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
}; //end of class def

#endif
