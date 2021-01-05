// program to convert lidar scans to images,
// find poster in corresponding image,
// and save resulting metric coordinates of checkerboard interior corners


#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d.hpp"
#include <iostream>
#include <fstream>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>




static const std::string OPENCV_WINDOW = "ImageWindow";
using namespace cv;
using namespace std;
using namespace Eigen;



Mat src, src_gray;

bool convert_pcd_to_image(string modifier) {
    return false;
}

/** @function main */
int main(int argc, char** argv) {
        ros::init(argc, argv, "find_poster_from_lidar"); //name this node

    ros::NodeHandle nh; 
    /// Load an image
    ROS_INFO("run this program from a directory that contains pcd data files from lidar scans");




    string modifier;
    cout<<"enter saved pcd filename modifier (e.g. for filename pcd1.txt, enter 1): ";
    cin>>modifier;
    cout<<"got modifier = "<<modifier<<endl;

    if (!convert_pcd_to_image(modifier)) {
        ROS_WARN("was not able to read and convert the specified pcd file");
    }
        
        return 0;
        
        
  /**/
    //src = imread("image16_rect_resized.png");
    //src = imread(fname.c_str());

    //if (!src.data) {
    //    cout<<"COULD NOT READ FILE"<<endl;
    //    return -1;
    //}

    //cout<<"got image size: "<<src.rows<<", "<<src.cols<<endl;
    /// Create a matrix of the same type and size as src (for dst)
    //dst.create(src.size(), src.type());

    cout<<"converting to grayscale"<<endl;
    cvtColor(src, src_gray, CV_BGR2GRAY);
    //src.copyTo(src, dst2);
    //dst2 = src;

    vector<Point2f> corners;
    Size patternsize(18,8); //interior number of corners
    cout<<"calling findChessboardCorners"<<endl;
    bool patternfound = findChessboardCorners( src_gray, patternsize, corners); //, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
    int n_corners = corners.size();
    cout<<"found "<<n_corners<<" corners"<<endl;
    //note: x,y output shows x must be along width, y is along height
    for (int i=0;i<n_corners;i++) {
       cout<<"corner: "<<corners[i].x<<", "<<corners[i].y<<endl;
    }

    if (!patternfound) {
      cout<<"did not find poster"<<endl;
      return 1;
    } 

    cout<<"calling cornerSubPix"<<endl;
    cornerSubPix(src_gray, corners, Size(11, 11), Size(-1, -1),
                     TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    cout<<"calling drawChessboardCorners"<<endl;
    drawChessboardCorners(src, patternsize, Mat(corners), patternfound);

    //save image:
    string processed_fname = "lidar_image"+modifier+".png";
    //imwrite(processed_fname,src);

    cout<<"calling imshow"<<endl;
    Mat src_smaller;
    resize(src, src_smaller, Size(src.cols/2,src.rows/2));
    imshow("result",src_smaller);

    /// Wait until user exits the program by pressing a key
    cout<<"calling waitKey(0)"<<endl;
    waitKey(0);
    cout<<"done; returning 0"<<endl;
    return 0;
}
