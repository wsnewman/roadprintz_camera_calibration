//program to find RoadPrintz checkerboard poster from image


#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d.hpp"
#include <iostream>
#include <fstream>


using namespace cv;
using namespace std;

typedef vector <double> record_t;
typedef vector <record_t> data_t;

istream& operator>>(istream& ins, record_t& record) {
	// make sure that the returned record contains only the stuff we read now
	record.clear();

	// read the entire line into a string (a CSV record is terminated by a newline)
	string line;
	getline(ins, line);
        cout<<"getline length: "<<line.size()<<endl;
                
                
	// now we'll use a stringstream to separate the fields out of the line
	stringstream ss(line);
	string field;
	while (getline(ss, field, ',')) {
		// for each field we wish to convert it to a double
		// (since we require that the CSV contains nothing but floating-point values)
		stringstream fs(field);
		double f = 0.0; // (default value is 0.0)
		fs >> f;

		// add the newly-converted field to the end of the record
		record.push_back(f);
	}

	// Now we have read a single line, converted into a list of fields, converted the fields
	// from strings to doubles, and stored the results in the argument record, so
	// we just return the argument stream as required for this kind of input overload function.
	return ins;
}

//-----------------------------------------------------------------------------
// Let's likewise overload the stream input operator to read a list of CSV records.
// This time it is a little easier, just because we only need to worry about reading
// records, and not fields.

istream& operator>>(istream& ins, data_t& data) {
	// make sure that the returned data only contains the CSV data we read here
	data.clear();

	// For every record we can read from the file, append it to our resulting data
	record_t record;
	while (ins >> record) {
		data.push_back(record);
	}

	// Again, return the argument stream as required for this kind of input stream overload.
	return ins;
}

bool read_image_pts(std::string fname,vector<Point2d> &image_pts) {
        
        ifstream infile(fname.c_str());
	if (!infile){ //The file couldn't be opened.
		cerr << "Error: file "<<fname.c_str()<<" could not be opened" << endl;
		return false;
	}        
        //vector<double> pt;
        //pt.resize(2);
        Point2d pt;
	// Here is the data we want.
	data_t data;

	// Here is the file containing the data. Read it into data.
	infile >> data;

	// Complain if something went wrong.
	if (!infile.eof()) {
		cout << "error reading file!\n";
		return false;
	}

	infile.close();       
        cout << "CSV file "<<fname<<" contains " << data.size() << " records.\n";
        int nrecords = (int) data.size();

        //image_pts.clear(); nope: append points
        for (int irecord=0;irecord<nrecords;irecord++) {
            pt.x = data[irecord][0];
            pt.y = data[irecord][1];
            image_pts.push_back(pt);
        }
        
        return true;
}

bool read_object_pts(std::string fname,vector<Point3d> &object_pts) {
        
        ifstream infile(fname.c_str());
	if (!infile){ //The file couldn't be opened.
		cerr << "Error: file "<<fname.c_str()<<" could not be opened" << endl;
		return false;
	}        

        Point3d pt;
        pt.z=0.0;
	// Here is the data we want.
	data_t data;

	// Here is the file containing the data. Read it into data.
	infile >> data;

	// Complain if something went wrong.
	if (!infile.eof()) {
		cout << "error reading file!\n";
		return false;
	}

	infile.close();       
        cout << "CSV file "<<fname<<" contains " << data.size() << " records.\n";
        int nrecords = (int) data.size();

        //object_pts.clear(); //nope; append points
        for (int irecord=0;irecord<nrecords;irecord++) {
            pt.x = data[irecord][0];
            pt.y = data[irecord][1];
            object_pts.push_back(pt);
        }
        
        return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "find_extrinsics"); //name this node
    vector<Point2d> imagePoints,imagePointsReprojected;
    vector<Point3d> objectPoints;
    Point2d imagePoint;
    Point3d objectPoint;
    
    cout<<"RUN ME FROM directory: ROS_WS/stella_calib_data/stella_lidar_data "<<endl;
    
    string fname = "image1_corners.csv";
    if(!read_image_pts(fname,imagePoints)) return 1;
    fname = "lidar_pts1_metric.csv";
    if(!read_object_pts(fname,objectPoints)) return 1;   
    
    fname = "image2_corners.csv";
    if(!read_image_pts(fname,imagePoints)) return 1;
    fname = "lidar_pts2_metric.csv";
    if(!read_object_pts(fname,objectPoints)) return 1;  

    fname = "image5_corners.csv";
    if(!read_image_pts(fname,imagePoints)) return 1;
    fname = "lidar_pts5_metric.csv";
    if(!read_object_pts(fname,objectPoints)) return 1;  
    
    fname = "image6_corners.csv";
    if(!read_image_pts(fname,imagePoints)) return 1;
    fname = "lidar_pts6_metric.csv";
    if(!read_object_pts(fname,objectPoints)) return 1;     

    fname = "image8_corners.csv";
    if(!read_image_pts(fname,imagePoints)) return 1;
    fname = "lidar_pts8_metric.csv";
    if(!read_object_pts(fname,objectPoints)) return 1;    

    /**/
    fname = "image11_corners.csv";
    if(!read_image_pts(fname,imagePoints)) return 1;
    fname = "lidar_pts11_metric.csv";
    if(!read_object_pts(fname,objectPoints)) return 1;   

    fname = "image13_corners.csv";
    if(!read_image_pts(fname,imagePoints)) return 1;
    fname = "lidar_pts13_metric.csv";
    if(!read_object_pts(fname,objectPoints)) return 1;   

    
    //intrinsics:
     double   fx = 1637.367343; 
     double   fy = 1638.139550;     
     //I get a better fit swapping cx and cy:
     double cy = 774.029343;  //hmm...I hope these are the right order!
     double cx = 1313.144667; //try swapping?
     
     Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
     cameraMatrix.at<double>(0,0) = fx;
     cameraMatrix.at<double>(1,1) = fy;
     cameraMatrix.at<double>(0,2) = cx;
     cameraMatrix.at<double>(1,2) = cy;
     
     Mat distCoeffs;
     Mat rvec,tvec;
     //see: https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#ga549c2075fac14829ff4a58bc931c033d
     solvePnP(objectPoints,imagePoints,cameraMatrix,distCoeffs,rvec,tvec);
     std::cout << "cameraMatrix : "<<endl << cameraMatrix << std::endl;

     // std::cout << "distCoeffs : " << distCoeffs << std::endl;

     std::cout << "Rotation vector : " << endl<< rvec << std::endl;

     std::cout << "Translation vector : " << endl<< tvec << std::endl;
     
     //compute reprojection error:
     projectPoints(objectPoints,rvec,tvec,cameraMatrix,distCoeffs,imagePointsReprojected);
     cout<<"image points: "<<endl<<imagePoints<<endl;
     cout<<"reprojections: "<<endl<<imagePointsReprojected<<endl;
     //Mat pts_diff = imagePointsReprojected-imagePoints;
     int npts = imagePoints.size();
     cout<<"npts = "<<npts<<endl;

     
    double err = norm(imagePointsReprojected,imagePoints);
     

     cout<<"rms err: "<<err/sqrt(npts)<<endl;
     cout<<"err each pt: "<<endl;
     int group=0;
     int nrecords=0;
     for (int i=0;i<npts;i++) {
         cout<< norm(imagePoints[i]-imagePointsReprojected[i])<<endl;
         nrecords++;
         if (nrecords>144) {
             group++;
             nrecords=0;
             cout<<"***********GROUP "<<group<<"*************"<<endl;
         }
     }
     
    //Affine3 xform(rvec, tvec);
     
     //EXPECTED something like:
     //vector from sys_ref_frame to camera_frame:   2.81156, 0, 2.87183;
     // R = [-1,0,0
     //      0, 1,0
     //      0, 0,-1]

    
    cv::Affine3d  T(rvec, tvec);
    //cout<<"affine: "<<endl<<T<<endl;
    cv::Matx33d R = T.rotation();
    cout<<"R: "<<endl<<R<<endl;
    cv::Vec3d trans = T.translation();
    cout<<"trans: "<<endl<<trans<<endl;
     cout<<"npts = "<<npts<<endl;
     cout<<"rms err: "<<err/sqrt(npts)<<endl;
    return 0;
}
