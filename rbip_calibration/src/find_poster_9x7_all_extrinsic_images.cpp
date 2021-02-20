//program to find RoadPrintz checkerboard poster from images
// this version has settings for image rotations at different heights


#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d.hpp"
#include <iostream>
#include <fstream>

//set max index for x, y, z to autoread images
int imax = 0; //indices range from 0 t/m 6
int jmax = 0;
int kmax = 4;
int rmax = 3;

int g_ans;

using namespace cv;
using namespace std;

string gen_fname(int i, int j, int k, int r) {
    stringstream ss;
    //ss << g_dir_name;
    ss<<"image_"<<k<<"_"<<j<<"_"<<i<<"_"<<r<<".png";
    string str = ss.str();
    return str;
}

string gen_fname_root(int i, int j, int k, int r) {
    stringstream ss;
    //ss << g_dir_name;
    ss<<"image_"<<k<<"_"<<j<<"_"<<i<<"_"<<r;
    string str = ss.str();
    return str;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "find_poster"); //name this node
    Mat src, src_gray;
    /// Load an image
    //cout<<"enter image filename, WITHOUT .png: ";
    string fname,fname_root;
    //cin>>fname_root;
    int i=0;
    int j=0;

           for (int k=0;k<=kmax;k++) {
               for (int r=0;r<=rmax;r++) {
                fname=gen_fname(i,j,k,r);
                fname_root = gen_fname_root(i,j,k,r);
          

                src = imread(fname.c_str());

    if (!src.data) {
        cout<<"COULD NOT READ FILE "<<fname<<endl;
        //return -1;
    }

    cout<<"got image size: "<<src.rows<<", "<<src.cols<<endl;
    /// Create a matrix of the same type and size as src (for dst)
    //dst.create(src.size(), src.type());
    //dst2 = src.clone();

    cout<<"converting to grayscale"<<endl;
    cvtColor(src, src_gray, CV_BGR2GRAY);

    vector<Point2f> corners;
    Size patternsize(8,6); //interior number of corners
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
    
    ofstream corners_file;
    string corners_filename =fname_root+"_corners.csv";
        corners_file.open(corners_filename.c_str(), ios::out | ios::trunc);
        
    for (int i=0;i<n_corners;i++) {
            cout<<"subpix corner: "<<corners[i].x<<", "<<corners[i].y<<endl;
                   
            corners_file << corners[i].x<<", "<<corners[i].y<<endl;          
        }
        corners_file.close();     
    
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
    //cout<<"calling waitKey(0)"<<endl;
    waitKey(1000);
    //cout<<"enter 1: ";
    //cin>>g_ans;
           
       
    }
           }
    
    cout<<"done; returning 0"<<endl;
    return 0;
}
