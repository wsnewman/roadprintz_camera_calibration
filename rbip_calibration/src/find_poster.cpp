//program to find RoadPrintz checkerboard poster from image
//SPECIALIZED for 18 keypoints x 8 keypoints


#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d.hpp"
#include <iostream>
#include <fstream>


using namespace cv;
using namespace std;

//check ordering of key points
//nominal pose has poster oriented horizontal with first row of corner points foreward and ordered port-to-starboard
//if in nominal pose, corner points are 0, 17, 126 and 143
//in this pose, should expect to find x-coord monotonically increasing from point 0 to point 17
//should also find y value of point 126 > y value of pt 0

    //extract just the 4 corner key points:
   // coarseImagePoints.push_back(tempImagePoints[0]); //aft/starboard
   // coarseImagePoints.push_back(tempImagePoints[17]);
   // coarseImagePoints.push_back(tempImagePoints[126]);
   // coarseImagePoints.push_back(tempImagePoints[143]);

//logic: ordering of key pts is 0,17,126,143
// in nominal pose, pt 0 is fore-port, pt17 is fore-starboard, pt126 is aft-port and pt143 is aft-starboard
// want to save these in order: fore-port, aft-port,aft-starboard, fore-starboard
// extract key pts as pt 1,2,3,4
// how to identify these:   
//   compute the centroid of these points
//   pts should lie in 4 quadrants: relative to centroid, fore-port has neg x, neg y
//   aft-port: neg x,  pos y
//   aft-starboard: pos x, pos y
//   fore-starboard: pos x, neg y

bool is_fore_port(Point2f pt, float mean_x, float mean_y) {
    if (  ((pt.x-mean_x)<0) && ((pt.y-mean_y)<0) ) return true;
    return false;
}

bool is_aft_port(Point2f pt, float mean_x, float mean_y) {
    if ( ((pt.x-mean_x)<0) && ((pt.y-mean_y)>0) ) { return true; }
    return false;
}

bool is_aft_starboard(Point2f pt, float mean_x, float mean_y) {
    if ( ((pt.x-mean_x)>0) && ((pt.y-mean_y)>0) ) return true;
    return false;
}

bool is_fore_starboard(Point2f pt, float mean_x, float mean_y) {
    if ( ((pt.x-mean_x)>0) && ((pt.y-mean_y)<0) ) return true;
    return false;
}

    
bool extract_and_order_corner_pts(vector<Point2f> corners, vector<Point2f> &ordered_4_corners)  {

    ordered_4_corners.clear();
    Point2f pt0,pt17,pt126,pt143;
    Point2f pt_fore_port,pt_aft_port,pt_aft_starboard,pt_fore_starboard;
    pt0 = corners[0];
    pt17=corners[17];
    pt126=corners[126];
    pt143=corners[143];
    
    float mean_x,mean_y;
    bool have_fore_port=false;
    bool have_aft_port=false;
    bool have_aft_starboard=false;
    bool have_fore_starboard=false;
    
    mean_x = (pt0.x+pt17.x+pt126.x+pt143.x)/4.0;
    mean_y = (pt0.y+pt17.y+pt126.y+pt143.y)/4.0;
    ROS_INFO("mean_x, mean_y = %f, %f",mean_x,mean_y);
    ROS_INFO("pt0.x, pt0,y = %f, %f",pt0.x,pt0.y);
    if( is_fore_port(pt0, mean_x, mean_y)) {
        pt_fore_port=pt0;
        have_fore_port=true;
        ROS_INFO("assigning pt0 to pt_fore_port");
    }
    else if (is_aft_port(pt0, mean_x, mean_y))  {
        pt_aft_port = pt0;
        have_aft_port=true;
        ROS_INFO("assigning pt0 to pt_aft_port");
    }
    else if (is_aft_starboard(pt0, mean_x, mean_y))  {
        pt_aft_starboard = pt0;
        have_aft_starboard=true;
        ROS_INFO("assigning pt0 to pt_aft_starboard");
        
    }
    else if (is_fore_starboard(pt0, mean_x, mean_y))  {
        pt_fore_starboard = pt0;
        have_fore_starboard=true;
                ROS_INFO("assigning pt0 to pt_fore_starboard");

    }    
    else {
        ROS_WARN("something is wrong! could not classify key corner point");
        return false;
    }
    
    //repeat for pt17:
    if( is_fore_port(pt17, mean_x, mean_y)) {
        pt_fore_port=pt17;
        have_fore_port=true;
    }
    else if (is_aft_port(pt17, mean_x, mean_y))  {
        pt_aft_port = pt17;
        have_aft_port=true;
    }
    else if (is_aft_starboard(pt17, mean_x, mean_y))  {
        pt_aft_starboard = pt17;
        have_aft_starboard=true;
    }
    else if (is_fore_starboard(pt17, mean_x, mean_y))  {
        pt_fore_starboard = pt17;
        have_fore_starboard=true;
    }    
    else {
        ROS_WARN("something is wrong! could not classify key corner point");
        return false;
    }    
    
    //repeat for pt 126    
    if( is_fore_port(pt126, mean_x, mean_y)) {
        pt_fore_port=pt126;
        have_fore_port=true;
    }
    else if (is_aft_port(pt126, mean_x, mean_y))  {
        pt_aft_port = pt126;
        have_aft_port=true;
    }
    else if (is_aft_starboard(pt126, mean_x, mean_y))  {
        pt_aft_starboard = pt126;
        have_aft_starboard=true;
    }
    else if (is_fore_starboard(pt126, mean_x, mean_y))  {
        pt_fore_starboard = pt126;
        have_fore_starboard=true;
    }    
    else {
        ROS_WARN("something is wrong! could not classify key corner point");
        return false;
    }    
    
    //finally, pt143:
    if( is_fore_port(pt143, mean_x, mean_y)) {
        pt_fore_port=pt143;
        have_fore_port=true;
    }
    else if (is_aft_port(pt143, mean_x, mean_y))  {
        pt_aft_port = pt143;
        have_aft_port=true;
    }
    else if (is_aft_starboard(pt143, mean_x, mean_y))  {
        pt_aft_starboard = pt143;
        have_aft_starboard=true;
    }
    else if (is_fore_starboard(pt143, mean_x, mean_y))  {
        pt_fore_starboard = pt143;
        have_fore_starboard=true;
    }    
    else {
        ROS_WARN("something is wrong! could not classify key corner point");
        return false;
    }      
    //make sure all 4 corners are identified:
    
    bool found_corners = have_fore_port&&have_aft_port&&have_aft_starboard&&have_fore_starboard;
    if (!found_corners) {
        ROS_WARN("something is wrong--did not identify all four corner key points");
        return false;
    }
    ordered_4_corners.push_back(pt_fore_port);
    ordered_4_corners.push_back(pt_aft_port);
    ordered_4_corners.push_back(pt_aft_starboard);
    ordered_4_corners.push_back(pt_fore_starboard);
    
    return true;
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "find_poster"); //name this node
    Mat src, src_gray;
    /// Load an image
    cout<<"enter image filename, WITHOUT .png: ";
    string fname_root;
    cin>>fname_root;
    string fname = fname_root+".png";
    src = imread(fname.c_str());

    if (!src.data) {
        cout<<"COULD NOT READ FILE"<<endl;
        return -1;
    }

    cout<<"got image size: "<<src.rows<<", "<<src.cols<<endl;
    /// Create a matrix of the same type and size as src (for dst)
    //dst.create(src.size(), src.type());
    //dst2 = src.clone();

    cout<<"converting to grayscale"<<endl;
    cvtColor(src, src_gray, CV_BGR2GRAY);

    vector<Point2f> corners;
    vector<Point2f> ordered_4_keypts;
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
    
    ofstream keypoints_file,corners_file;
    string corners_filename =fname_root+"_corners.csv";
    string keypoints_filename =fname_root+"_keypoints.csv";
    
    //keypoints_filename
    
        keypoints_file.open(keypoints_filename.c_str(), ios::out | ios::trunc);
        
    for (int i=0;i<n_corners;i++) {
            cout<<"subpix corner: "<<corners[i].x<<", "<<corners[i].y<<endl;
                   
            keypoints_file << corners[i].x<<", "<<corners[i].y<<endl;          
        }
        keypoints_file.close();   
        
   //find the 4 corners and save them in the specific order: fore_port,aft_port,aft_starboard,fore_starboard
    corners_file.open(corners_filename.c_str(), ios::out | ios::trunc);

    if( extract_and_order_corner_pts(corners, ordered_4_keypts))  {
        ROS_INFO("saving 4 keypts in desired order");
       //store points in desired order, fore_port, aft_port,aft_starboard,fore_starboard
        for (int ipt=0;ipt<4;ipt++) {
            corners_file << ordered_4_keypts[ipt].x<<", "<<ordered_4_keypts[ipt].y<<endl; 
        }    
   }        
    else {
        ROS_WARN("something went wrong with identifying/ordering 4 key points!");
    }
    
    cout<<"calling drawChessboardCorners"<<endl;
    drawChessboardCorners(src, patternsize, Mat(corners), patternfound);

    //save image:
    string processed_fname = "processed_"+fname;
    imwrite(processed_fname,src);

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
