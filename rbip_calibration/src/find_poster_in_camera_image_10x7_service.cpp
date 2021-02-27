//find_poster_in_camera_image_10x7_service.cpp
// wsn, 2/27/21
//program to find RoadPrintz checkerboard poster from image
//this version: offers a SERVICE
//SPECIALIZED for  10 keypoints x 7 keypoints
//assumes camera image, not lidar image
// service request should include image name prefix (e.g. image1_)
// this node will assume the image of interest will be found in: calibration_temp_files,
// with names prefix_image_rect.png and prefix_virtual_image.png


#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d.hpp"
#include <iostream>
#include <fstream>

#include <camera_parameters/virtual_cam_params.h> //need this for KPIX value

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <std_srvs/Trigger.h>
#include <rbip_calibration/CamPosterCalibTestSrv.h>


//static const std::string OPENCV_WINDOW = "ImageWindow";
using namespace cv;
using namespace std;
using namespace Eigen;
int g_ans;

static double EXPECTED_NPTS=70; //customized for 10x7 poster, which has 11 columns and 8 rows of squares

static double SQUARE_SIZE_METRIC=0.1017; //set known size of squares in poster; MATCH THIS TO find_poster_v2.cpp
static double NCOLS=11;
static double NROWS=8;


//bool find_poster_service_callback(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response) {
bool find_poster_service_callback(rbip_calibration::CamPosterCalibTestSrvRequest& request, rbip_calibration::CamPosterCalibTestSrvResponse& response) {
    
    ROS_WARN("find_poster_service_callback");
    Mat src, src_gray;  
    string root_fname = request.fname;
    cout<<"input fname: "<<root_fname<<endl;
           
    string input_image_name, keypoints_filename,markup_filename,image_filename;
    
        //grid_filename =root_fname+"_gridpoints.csv";
        keypoints_filename = root_fname+"_keypoints.csv";
        markup_filename = root_fname+"_markup.png";
        image_filename = root_fname+".png";
    //cout<<"using grid_filename: "<<grid_filename<<endl;
    src = imread(image_filename.c_str());

    if (!src.data) {
        ROS_ERROR_STREAM("COULD NOT READ FILE "<<image_filename.c_str()<<endl);
        response.success = false;
        return true;
    }

    ROS_INFO_STREAM("got image size: "<<src.rows<<", "<<src.cols<<endl);
    cout<<"converting to grayscale"<<endl;
    cvtColor(src, src_gray, CV_BGR2GRAY);
    
    vector<Point2f> corners;
    //vector<Point2f> ordered_4_keypts;
    Size patternsize(NCOLS-1,NROWS-1); //interior number of corners
    ROS_INFO("calling findChessboardCorners");
    bool patternfound = findChessboardCorners( src, patternsize, corners); //, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
    if (!patternfound) {
      ROS_ERROR("did not find poster");
        response.success = false;
        return true;
    }  
    
    int n_corners = corners.size();
    if (n_corners!=EXPECTED_NPTS) {
        ROS_ERROR("something wrong; pts mismatch; n_corners found = %d",n_corners);
        response.success = false;
        return true;        
        
    }
    ROS_INFO_STREAM("found "<<n_corners<<" corners"<<endl);
    //note: x,y output shows x must be along width, y is along height
    //for (int i=0;i<n_corners;i++) {
       //cout<<"corner: "<<corners[i].x<<", "<<corners[i].y<<endl;
    //}
    
    cout<<"calling cornerSubPix"<<endl;
    cornerSubPix(src_gray, corners, Size(11, 11), Size(-1, -1),
                     TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));


   
        
    ofstream keypoints_file; //,corners_file,gridpts_file,rbif_points_file;
    //string corners_filename ="corners.csv";
    //string keypoints_filename ="lidar_keypoints.csv";
    //string grid_filename =g_data_dir+"/"+root_fname+"/lidar_gridpoints.csv";
        
    //    gridpts_file.open(grid_filename.c_str(), ios::out | ios::trunc);
    
        keypoints_file.open(keypoints_filename.c_str(), ios::out | ios::trunc);
               
    for (int i=0;i<n_corners;i++) {                   
            keypoints_file <<i<<", "<< corners[i].x<<", "<<corners[i].y<<endl;          
        }
        keypoints_file.close();   
        
    cout<<"calling drawChessboardCorners"<<endl;
    //drawChessboardCorners(src_gray, patternsize, Mat(corners), patternfound);
    drawChessboardCorners(src, patternsize, Mat(corners), patternfound);

    ROS_INFO_STREAM("saving processed image to"<<markup_filename<<endl);
    //imwrite(markup_filename,src_gray);  
    imwrite(markup_filename,src);   

    response.success = true;
    return true;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "find_poster_in_cam_image"); //name this node
    ros::NodeHandle nh;

    Mat src, src_gray;
    /// Load an image
    //cout<<"enter image filename, WITHOUT .png: ";
    
    /*
    string ros_ws_path = getenv("ROS_WORKSPACE");
    //string path_to_pointcloud_image= ros_ws_path + "/src/roadprintz_ros_code/pcl_utils/image_from_pointcloud.png";
    //string path_to_pointcloud_image= "image_from_pointcloud.png"; //change to using current directory
    string path_to_pointcloud_image= ros_ws_path +"/calibration_temp_files/image_from_pointcloud.png"; //change to using current directory
    
    g_path_to_pointcloud_image = path_to_pointcloud_image;
    //g_path_to_markup_image = ros_ws_path +"/calibration_temp_files/image_from_pointcloud_markup.png";
    
    g_data_dir = ros_ws_path +"/calibration_temp_files/";
    //g_keypoints_filename =ros_ws_path +"/calibration_temp_files/lidar_keypoints.csv";
    //g_grid_filename =ros_ws_path +"/calibration_temp_files/lidar_gridpoints.csv";
    g_rbif_pts_filename = ros_ws_path +"/calibration_temp_files/estimated_rbif_keypoints.csv";
    */
    ros::ServiceServer make_image_service = nh.advertiseService("find_poster_in_image_service",find_poster_service_callback); 

    ros::spin();
    return 0;
}
