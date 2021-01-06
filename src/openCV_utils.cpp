//program to find RoadPrintz checkerboard poster from image
#include <roadprintz_camera_calibration/openCV_utils.h>

/*
#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d.hpp"
#include <iostream>
#include <fstream>
*/

//this version is not part of class, and it takes args as Mat instead of filenames
//should fold this into openCV_utils for code re-use instead of redundancy

bool find_poster_in_image(Mat input_image, int n_keypts_wide, int n_keypts_high, bool &preferred_orientation, double &x_upper_left, double &y_upper_left, Mat &output_image) {
    Mat src_gray;
    output_image = input_image.clone(); 
    ROS_INFO("got image size: %d, %d",input_image.rows,input_image.cols);

    ROS_INFO("converting to grayscale");
    cvtColor(input_image, src_gray, CV_BGR2GRAY);

    vector<Point2f> corners;
    Size patternsize(n_keypts_wide,n_keypts_high); //interior number of corners
    //cout<<"calling findChessboardCorners"<<endl;
    bool patternfound = findChessboardCorners( src_gray, patternsize, corners); //, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
    if (!patternfound) {
      ROS_ERROR("did not find poster"); //<<endl;
      return false;
    } 





    //cout<<"calling cornerSubPix"<<endl;
    cornerSubPix(src_gray, corners, Size(11, 11), Size(-1, -1),
                     TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    
    int n_corners = corners.size();
    ROS_INFO("found %d corners",n_corners);
    //note: x,y output shows x must be along width, y is along height
    for (int i=0;i<n_corners;i++) {
       ROS_INFO_STREAM("corner: "<<corners[i].x<<", "<<corners[i].y<<endl);
    }    
    
    //if intend to use poster coordinates to match up with robot laser-pointer coords, then we care
    // about the order of the key points: monotonically increasing x values along first row n_keypts_wide wide,
    // and monotonically increasing y values for subsequent rows
    //check that pixel x values are monotonically increasing:
    ROS_INFO("testing if x values are monotonically increasing along first row: ");
    preferred_orientation=true;
    bool xvals_increasing = true;
    for (int ix=1;ix<n_keypts_wide;ix++) {
        double x_prev = corners[ix-1].x;
        double x_next = corners[ix].x;
        double x_inc = x_next-x_prev;
        ROS_INFO("x_inc = %f",x_inc);
        if (x_inc<0) {
            xvals_increasing = false;
            ROS_WARN("x values along first row are not monotonically increasing for %d key points",n_keypts_wide);
        }
    }
    if (!xvals_increasing) preferred_orientation=false;
    
    //check that y values are monotonically increasing:
    ROS_INFO("testing if y values are monotonically increasing in subsequent rows: ");
    bool yvals_increasing = true;
    for (int iy=1;iy<n_keypts_high;iy++) {
        int index_prev = (iy-1)*n_keypts_wide;
        int index_next = iy*n_keypts_wide;
        double y_prev = corners[index_prev].y;
        double y_next = corners[index_next].y;
        double y_inc = y_next-y_prev;
        ROS_INFO("y_inc = %f",y_inc);
        if (y_inc<0) {
            yvals_increasing = false;
            ROS_WARN("y values are not monotonically increasing for %d rows",n_keypts_high);
        }
    }    
    if (!yvals_increasing) preferred_orientation=false;

    x_upper_left=corners[0].x;
    y_upper_left=corners[0].y;
    
    
  
    
    cout<<"calling drawChessboardCorners"<<endl;
    drawChessboardCorners(output_image, patternsize, Mat(corners), patternfound);

    return true;
}


//OpenCvUtils::OpenCvUtils() {
//    ROS_INFO("OpenCvUtils constructor");
//}

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

bool OpenCvUtils::find_poster(string path, string fname_image_root,int n_keypts_wide, int n_keypts_high, bool &preferred_orientation, double &x_upper_left, double &y_upper_left) {
    Mat src, src_gray;
    // Load an image
    string fname_image_w_path = path+"/"+fname_image_root+".png";
    src = imread(fname_image_w_path.c_str());

    if (!src.data) {
        ROS_ERROR("find_poster: COULD NOT READ IMAGE FILE %s",fname_image_w_path.c_str());
        return false;
    }
    ROS_INFO("got image size: %d, %d",src.rows,src.cols);

    ROS_INFO("converting to grayscale");
    cvtColor(src, src_gray, CV_BGR2GRAY);

    vector<Point2f> corners;
    Size patternsize(n_keypts_wide,n_keypts_high); //interior number of corners
    //cout<<"calling findChessboardCorners"<<endl;
    bool patternfound = findChessboardCorners( src_gray, patternsize, corners); //, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
    if (!patternfound) {
      ROS_ERROR("did not find poster"); //<<endl;
      return false;
    } 





    //cout<<"calling cornerSubPix"<<endl;
    cornerSubPix(src_gray, corners, Size(11, 11), Size(-1, -1),
                     TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    
    int n_corners = corners.size();
    ROS_INFO("found %d corners",n_corners);
    //note: x,y output shows x must be along width, y is along height
    for (int i=0;i<n_corners;i++) {
       ROS_INFO_STREAM("corner: "<<corners[i].x<<", "<<corners[i].y<<endl);
    }    
    
    //if intend to use poster coordinates to match up with robot laser-pointer coords, then we care
    // about the order of the key points: monotonically increasing x values along first row n_keypts_wide wide,
    // and monotonically increasing y values for subsequent rows
    //check that pixel x values are monotonically increasing:
    ROS_INFO("testing if x values are monotonically increasing along first row: ");
    preferred_orientation=true;
    bool xvals_increasing = true;
    for (int ix=1;ix<n_keypts_wide;ix++) {
        double x_prev = corners[ix-1].x;
        double x_next = corners[ix].x;
        double x_inc = x_next-x_prev;
        ROS_INFO("x_inc = %f",x_inc);
        if (x_inc<0) {
            xvals_increasing = false;
            ROS_WARN("x values along first row are not monotonically increasing for %d key points",n_keypts_wide);
        }
    }
    if (!xvals_increasing) preferred_orientation=false;
    
    //check that y values are monotonically increasing:
    ROS_INFO("testing if y values are monotonically increasing in subsequent rows: ");
    bool yvals_increasing = true;
    for (int iy=1;iy<n_keypts_high;iy++) {
        int index_prev = (iy-1)*n_keypts_wide;
        int index_next = iy*n_keypts_wide;
        double y_prev = corners[index_prev].y;
        double y_next = corners[index_next].y;
        double y_inc = y_next-y_prev;
        ROS_INFO("y_inc = %f",y_inc);
        if (y_inc<0) {
            yvals_increasing = false;
            ROS_WARN("y values are not monotonically increasing for %d rows",n_keypts_high);
        }
    }    
    if (!yvals_increasing) preferred_orientation=false;

    x_upper_left=corners[0].x;
    y_upper_left=corners[0].y;
    
    
    ofstream corners_file;
    //string corners_filename =fname_root+"_corners.csv";
    string fname_corners =  path+"/"+fname_image_root+"_corners.csv";

        corners_file.open(fname_corners.c_str(), ios::out | ios::trunc);
        
    for (int i=0;i<n_corners;i++) {
           // cout<<"subpix corner: "<<corners[i].x<<", "<<corners[i].y<<endl;
                   
            corners_file << corners[i].x<<", "<<corners[i].y<<endl;          
        }
        corners_file.close();     
    
    cout<<"calling drawChessboardCorners"<<endl;
    drawChessboardCorners(src, patternsize, Mat(corners), patternfound);

    //save image:
    string processed_image_fname = path+"/"+fname_image_root+"_processed.png";
    imwrite(processed_image_fname,src);
    
    return true;
}




//class image_converter:
/*
class ImageTransportUtil
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  //void ImageConverterMouseCB(int event, int x, int y, int flags, void* userdata);

public: */
  ImageTransportUtil::ImageTransportUtil()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe(g_input_image_topic.c_str(), 1,
      &ImageTransportUtil::imageCb, this);

    image_pub_ = it_.advertise(g_output_image_topic.c_str(), 1);

    cv::namedWindow(OPENCV_WINDOW);
    //cv::namedWindow(OPENCV_ZOOM_WINDOW);
         //set the callback function for any mouse event
     //setMouseCallback(OPENCV_WINDOW, ImageConverterMouseCB, NULL);
     //setMouseCallback(OPENCV_WINDOW, ImageConverterMouseCB, NULL);
     //setMouseCallback(OPENCV_ZOOM_WINDOW, ImageConverterZoomedMouseCB, NULL);
     //    minimal_subscriber_ = nh_.subscribe("example_class_input_topic", 1, &ExampleRosClass::subscriberCallback,this);  

  }




//do transform inside this callback
  void ImageTransportUtil::imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    Mat dst_smaller;
    Mat src = cv_ptr->image;
    g_src = cv_ptr->image;
    ROS_INFO("imageCb: received new image");
    
    if (!g_got_new_image) {
        g_copy_of_image = g_src.clone();
        g_copy_of_image_markup= g_src.clone();
        
        g_got_new_image=true;

        ROS_INFO("imageCb: processed new image");
        //find_poster_in_image(Mat input_image, int n_keypts_wide, int n_keypts_high, bool &preferred_orientation, double &x_upper_left, double &y_upper_left, Mat &output_image)
        //double g_x_pix_upper_left,g_y_pix_upper_left;

        g_found_poster = find_poster_in_image(g_copy_of_image, g_n_keypts_wide, g_n_keypts_high, g_preferred_orientation, g_x_pix_upper_left, g_y_pix_upper_left, g_copy_of_image_markup);
    } //end of callback fnc
  }

