//12/31/20: modify find_extrinsics_v2 to use LIDAR scans
//NOT DONE YET!!

//program to find camera extrinsics based on checkerboard key points
//On 12/30/20, obtained key points for 4 poster poses.
//recorded respective images in: roadprintz_ws/stella_calib_data/rbip_calib_12_30_20
//also recorded key points (outermost inner intersections of checkerboard, i.e. pts 1, 18, 127, 144)
// by steering robot w/ laser pointer to these intersections
// this data is recorded in files: image1_key_pts_metric.csv through image4_key_pts_metric.csv
// Grids fit to the (pretransformed) images have keypoints output in files: image1_pretransformed_corners.csv
// through image4_pretransformed_corners.csv

//CAREFUL with ordering: in images, top-left corner (pt 1) corresponds to fore/port.  MUST list robot-pointer based points
// in order: fore/port, fore/starboard, aft/port, aft/starboard to get correspondence correct.
// This assumes the checkerboard poster fit has "top" line (in processed image) as red line, from left to right
// (bottom line is also red, from left to right, but first line segues by colors: ROYGBIV)

//using Octave program interpret_key_pts.m, length of poster is 1.7147m (for 17 squares) and width is 0.70543 (for 7 squares)
// avg square dimension = 0.10086  (based on lengths/widths from 16 robot/laser-pointer points in RBIP frame)

//want to find extrinsic parameters of camera (in camera pose) with respect to RBIP frame


#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/calib3d.hpp"
#include <iostream>
#include <fstream>

#include <xform_utils/xform_utils.h>
#include <eigen3/Eigen/src/Geometry/Transform.h>

using namespace cv;
using namespace std;

typedef vector <double> record_t;
typedef vector <record_t> data_t;
XformUtils xformUtils;

//MAGIC NUMBERS:
    //intrinsics:

     double   fx = 1637.367343; 
     double   fy = 1638.139550;   
     //double f = 1638.0; // can't tell which is which
     //I get a better fit swapping cx and cy:
     double ci = 774.029343;  //hmm...I hope these are the right order!
     double cj = 1313.144667; //try swapping?  OR...swap input values of image coords
     double cx = cj;
     double cy = ci;
int g_ans;

//these get changed in call to get_hardcoded_affine_cam_wrt_sys
//vary params inside this fnc for search for optimal extrinsic fit
         double g_dsearch_trans = 0.0;
         double g_dsearch_rot = 0.0; 
     

istream& operator>>(istream& ins, record_t& record) {
	// make sure that the returned record contains only the stuff we read now
	record.clear();

	// read the entire line into a string (a CSV record is terminated by a newline)
	string line;
	getline(ins, line);
        //cout<<"getline length: "<<line.size()<<endl;
                
                
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

Eigen::Matrix3d Roty(double phi_y) {
    Eigen::Matrix3d Roty;
    Eigen::Vector3d ay;
    ay << 0, 1, 0;
    Roty = Eigen::AngleAxisd(phi_y, ay);
    return Roty;
}

Eigen::Matrix3d Rotx(double phi_x) {
    Eigen::Matrix3d Rotx;
    Eigen::Vector3d ax;
    ax << 1, 0, 0;
    Rotx = Eigen::AngleAxisd(phi_x, ax);
    return Rotx;
}

Eigen::Matrix3d Rotz(double phi_z) {
    Eigen::Matrix3d Rotz;
    Eigen::Vector3d az;
    az << 0, 0, 1;
    Rotz = Eigen::AngleAxisd(phi_z, az);
    return Rotz;
}

void repair_R(Eigen::Matrix3d &R) {

  //Eigen::Matrix3d R_repaired = R;
  Eigen::Vector3d xvec,yvec,zvec;
  zvec= R.col(2);
  zvec = zvec/(zvec.norm());
  xvec = R.col(0);
  xvec = xvec- (xvec.transpose()*zvec)*zvec;
  xvec = xvec/xvec.norm();
  yvec = zvec.cross(xvec);
  R.col(0) = xvec;
  R.col(1) = yvec;
  R.col(2) = zvec;

}

Eigen::Affine3d compute_correction_transform(double dx, double dy, double dz, double dphix, double dphiy, double dphiz) {
    Eigen::Affine3d affine_correction;
    affine_correction.linear() = Rotx(dphix)*Roty(dphiy)*Rotz(dphiz);
    Eigen::Vector3d dvec;
    dvec << dx, dy, dz;
    affine_correction.translation() = dvec;
    //ROS_INFO("correction transform: ");
    //xformUtils.printAffine(g_affine_correction_camera_frame_wrt_sys);
    return affine_correction;
}

Eigen::Affine3d get_hardcoded_affine_sys_wrt_cam(void) {
	Eigen::Affine3d hardcoded_affine_sys_wrt_cam;

	Eigen::Vector3d trans;
	//from Tom, 10/26/20:
	// -0.996831  0.0451598  0.0654806 -0.0412742
	// 0.0629916 -0.0544956   0.996525   -2.75349
	// 0.0485713   0.997492  0.0514783    2.60876
	//         0          0          0          1
	trans<<  -0.041274, -2.753494, 2.608757;
        
	Eigen::Matrix3d R;
	Eigen::Vector3d xvec_sys_wrt_cam,yvec_sys_wrt_cam,zvec_sys_wrt_cam;
	xvec_sys_wrt_cam <<-0.996831,0.0629916,0.0485713;
	yvec_sys_wrt_cam <<0.0451598,-0.0544956,0.997492;
	zvec_sys_wrt_cam <<0.0654806,0.996525,0.0514783;

	R.col(0) = xvec_sys_wrt_cam;
	R.col(1) = yvec_sys_wrt_cam;
	R.col(2) = zvec_sys_wrt_cam;
	hardcoded_affine_sys_wrt_cam.linear() = R;
	hardcoded_affine_sys_wrt_cam.translation() = trans;

	return hardcoded_affine_sys_wrt_cam;

}

// convention:  viewing a camera scene (as we have it displayed),
//  the camera x-axis points to the right = u axis (= increasing columns, j)
//  the camera y-axis points down in the image, parallel to the v-axis (= increasing row index, i)
//  the camera z-axis points OUT of the camera, i.e. INTO the image

//  comparing this to the sys_ref_frame, the camera is roughly aligneed such that:
//  the camera x-axis is parallel to the system y axis
//  the camera y-axis is parallel to the system x axis
//  the camera z-axis is antiparallel to the system z axis

//note OpenCV convention on pixels is to express coordinates in (u,v), i.e. (col,row) order.
//the datafiles for calibration have the pixel coords in this order

//new 12/30/20: this is actually with respect to the RBIP frame, since the robot laser-pointer points are in the RBIP frame
Eigen::Affine3d get_hardcoded_affine_cam_wrt_sys(void) {
	Eigen::Affine3d hardcoded_affine_cam_wrt_sys;

	Eigen::Vector3d trans;
	Eigen::Matrix3d R;
	Eigen::Vector3d xvec_cam_wrt_sys,yvec_cam_wrt_sys,zvec_cam_wrt_sys;
        
        g_dsearch_trans = 0.0002;
        g_dsearch_rot = 0.0002;    
         
    //note: camera origin is about 2.885 meters high, and about 2.768m aft of rear wheels
    //   camera origin is nearly along vehicle centerline, but shifted about 7mm to starboard         

//trans<<1.6,  0.0109847,    2.8659 ;   
//trans<<1.66624, -0.15308,   2.82942; //40
//trans<<1.59124, -0.359786,      2.85; //33
//trans<<1.60812, -0.136404,   2.86453 ;//20
//trans<<1.53347, -0.00593869,     2.91676; //70
        
        
//trans<<1.4947, -0.0602015,    2.92583; //58
//trans<<1.63878, -0.166147,   2.88813; //48
        // trans<<1.53278, 0.00472392,    2.92286; //67
         //trans<<1.49399, -0.0496564,    2.92213;//55
//trans<<1.57156, -0.10241,  2.89981;//45
trans<<1.44604, -0.00440006, 2.86339;//0.918912
R<<0.0561968, 0.995697, 0.0736899,
    0.997359, -0.0593846, 0.0418105,
    0.0460069, 0.0711456, -0.996403;
   
  /* 
   R<< 0, 1, 0,
       1, 0, 0,
       0, 0, -1;
   */
         


     
     
    //note: camera x-axis is ~parallel to system y-axis
    //      camera y-axis is ~parallel to system x-axis
    //      camera z-axis is ~antiparallel to system z-axis
    

	hardcoded_affine_cam_wrt_sys.linear() = R;
	hardcoded_affine_cam_wrt_sys.translation() = trans;

	return hardcoded_affine_cam_wrt_sys;

}


//given corresponding points in camera pixels, imagePoints, and absolute (metric) coords w/rt sys_ref_frame (objectPoints)
//compute projections and compare to get pixel rms errors of calibration


void project_point(Eigen::Affine3d affine_sys_wrt_cam, Eigen::Vector3d objectPoint,Eigen::Vector2d imagePoint,Eigen::Vector2d &imagePointReproj) {
    Eigen::Vector3d object_pt_wrt_cam;
    //cout<<"pt wrt sys: "<<objectPoint.transpose()<<endl;
    object_pt_wrt_cam = affine_sys_wrt_cam*objectPoint;
    //cout<<"pt wrt cam: "<<object_pt_wrt_cam.transpose()<<endl;
    
 
    imagePointReproj[0] = cx+object_pt_wrt_cam[0]/object_pt_wrt_cam[2]*fx;
    imagePointReproj[1] = cy+object_pt_wrt_cam[1]/object_pt_wrt_cam[2]*fy;
    
    //cout<<"computed pt in i,j: "<<imagePointReproj.transpose()<<endl;
}



double   attempt_fit_points(Eigen::Affine3d affine_sys_wrt_cam,vector<Point2d> imagePoints,vector<Point3d> objectPoints) {
    
    Eigen::Vector3d objectPoint;
    Eigen::Vector2d imagePoint,imagePointReproj;
    int npts = objectPoints.size();
    double sum_sqd_err=0;
    for (int ipt=0;ipt< npts;ipt++) {
        //convert point to Eigen type:
        objectPoint[0]= objectPoints[ipt].x;
        objectPoint[1]= objectPoints[ipt].y;
        objectPoint[2]= objectPoints[ipt].z;
        //imagePoint[0] = imagePoints[ipt].x;
        //imagePoint[1] = imagePoints[ipt].y;
        // try swapping i and j: DEBUG
        imagePoint[0] = imagePoints[ipt].x; //this coordinate increases as raster along a ~horizontal line of checkerboard
        imagePoint[1] = imagePoints[ipt].y; // i.e., the x value is the column index and the y value is the row index
                                            // to be consistent w/ OpenCV, these should be called (u,v), not (x,y)
        
        //cout<<endl<<"actual image pt: "<<imagePoint.transpose()<<endl;
        
        project_point(affine_sys_wrt_cam,objectPoint,imagePoint,imagePointReproj);
        sum_sqd_err+= (imagePoint-imagePointReproj).transpose()*(imagePoint-imagePointReproj);
    }
    double rms_err  = sqrt(sum_sqd_err/npts);
    //ROS_INFO("got rms error of %f using affine cam/sys of: ",rms_err);
    //xformUtils.printAffine(affine_sys_wrt_cam.inverse());
    return rms_err;
    
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
            pt.x = data[irecord][0]; //data points are in (u,v) order, i.e. (col, row), per OpenCV convention
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

//convert between Eigen objects and OpenCV Points
Eigen::Vector3d pt3d_to_Eigen3d(Point3d pt3d) {
    Eigen::Vector3d eigen_v3d;
    eigen_v3d[0]=pt3d.x;
    eigen_v3d[1]=pt3d.y;
    eigen_v3d[2]=pt3d.z;
    return eigen_v3d;
}

Point3d Eigen3d_to_pt3d(Eigen::Vector3d eigen_v3d) {
    Point3d pt3d;
    pt3d.x=eigen_v3d[0];
    pt3d.y=eigen_v3d[1];
    pt3d.z=eigen_v3d[2];
    return pt3d;
}

//given the 4 corner points, (pts 1, 18, 126, 144), interpolate these to create other internal key points of poster
//assumes 144 key points for 18x8 interior points
//need to scan from aft/starboard to aft/port,
//then raster from starboard to port, increasingly fore
void interpolate_key_points(vector<Point3d> coarseObjectPoints,vector<Point3d> &objectPoints)  {
    //hard coded for 18x8 points, assuming poster is landscape-mode view
    int npts_length = 18;
    int npts_width = 8;
    Eigen::Vector3d pt1,pt18,pt127,pt144,pt_left,pt_right;
    Eigen::Vector3d pt;
    //pt names start counting from 1, not 0, consistent w/ spreadsheet rows
    Point3d pt3d_1,pt3d_18,pt3d_127,pt3d_144,pt3d_left,pt3d_right,pt3d;
    //I saved these in the order:  fore/port, fore/starboard, aft/port, aft/starboard

    pt3d_1 = coarseObjectPoints[0];
    pt3d_18 = coarseObjectPoints[1];
    pt3d_127 = coarseObjectPoints[2];
    pt3d_144 = coarseObjectPoints[3];
    pt1 = pt3d_to_Eigen3d(pt3d_1);
    pt18 = pt3d_to_Eigen3d(pt3d_18);
    pt127 = pt3d_to_Eigen3d(pt3d_127);
    pt144 = pt3d_to_Eigen3d(pt3d_144);
    
    cout<<"pt1:   "<<pt1.transpose()<<endl;
    cout<<"pt18:  "<<pt18.transpose()<<endl;
    cout<<"pt127: "<<pt127.transpose()<<endl;
    cout<<"pt144: "<<pt144.transpose()<<endl;

    
    for (int i=0;i<npts_width;i++) {
        pt_left = pt1+ i*(pt127-pt1)/(npts_width-1); //there are 7 intervals between the 8 points along the poster width
        pt_right= pt18+i*(pt144-pt18)/(npts_width-1);    
        //cout<<"row "<<i<<endl;
        //cout<<"pt_left:  "<<pt_left.transpose()<<endl;
        //cout<<"pt_right: "<<pt_right.transpose()<<endl;
        for (int j=0;j<npts_length;j++) {
            pt = pt_left+j*(pt_right-pt_left)/(npts_length-1); //17 intervals between the 18 points along the poster width
            //cout<<"pt interp: "<<pt.transpose()<<endl;
            pt3d=Eigen3d_to_pt3d(pt);
            objectPoints.push_back(pt3d);            
        }
    }
    //cout<<"enter 1: ";
    //cin>>g_ans;
    
}

void save_to_csv(std::string fname,vector<Point3d> objectPoints) {
        int npts = objectPoints.size();
        ofstream outfile;
        outfile.open(fname.c_str(), ios::out | ios::trunc);        
        //vector<int> intensity_ints;
        //vector<float> intensity_floats;
        //intensity_floats = scan_msgs[0].intensities;
        //int npings = scan_msgs[0].intensities.size(); //intensity_floats.size();
        ROS_INFO("saving %d points to csv file",npts);
        //intensity_ints.resize(nscans)
        Point3d pt;
        for (int ipt=0;ipt<npts;ipt++) {
            pt = objectPoints[ipt];
                outfile<<pt.x<<", "<<pt.y<<", "<<pt.z<<endl;
                        
        }
        outfile.close(); 
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "find_extrinsics"); //name this node
    vector<Point2d> imagePoints,imagePointsReprojected,coarseImagePoints,tempImagePoints;
    vector<Point3d> objectPoints,coarseObjectPointsAll,coarseObjectPoints1,coarseObjectPoints2,coarseObjectPoints3,coarseObjectPoints4;
    Point2d imagePoint;
    Point3d objectPoint;
    
        //choose search for about 2min, i.e. test center and +/- three samples either side
    //     double dsearch_trans = 0.01;
    //     double dsearch_rot = 0.01;    
    
    cout<<"RUN ME FROM directory: ROS_WS/stella_calib_data/rbip_calib_12_30_20"<<endl;
    string fname;
    //image 1:
     /*  */
    fname = "image1_pretransformed_corners.csv";
    if(!read_image_pts(fname,imagePoints)) return 1;
    
    //read in points again, extracting just the 4 corners for coarse key points
    tempImagePoints.clear();
    read_image_pts(fname,tempImagePoints);
    
    //extract just the 4 corner key points:
    coarseImagePoints.push_back(tempImagePoints[0]); //aft/starboard
    coarseImagePoints.push_back(tempImagePoints[17]);
    coarseImagePoints.push_back(tempImagePoints[126]);
    coarseImagePoints.push_back(tempImagePoints[143]);
    
    fname = "image1_key_pts_metric.csv";
    //coarseObjectPoints.clear();
    if(!read_object_pts(fname,coarseObjectPointsAll)) return 1;   
    read_object_pts(fname,coarseObjectPoints1);
    interpolate_key_points(coarseObjectPoints1,objectPoints);//tack on object1 pts to objectPoints


    //image 2:  append more points to imagePoints and to coarseImagePoints and to coarseObjectPoints
    fname = "image2_pretransformed_corners.csv";
    if(!read_image_pts(fname,imagePoints)) return 1;
    //read in points again, extracting just the 4 corners for coarse key points
    tempImagePoints.clear();
    read_image_pts(fname,tempImagePoints);
    
    //extract just the 4 corner key points:
    coarseImagePoints.push_back(tempImagePoints[0]); //aft/starboard
    coarseImagePoints.push_back(tempImagePoints[17]);
    coarseImagePoints.push_back(tempImagePoints[126]);
    coarseImagePoints.push_back(tempImagePoints[143]);    
    
    fname = "image2_key_pts_metric.csv";
    //coarseObjectPoints.clear();
    if(!read_object_pts(fname,coarseObjectPointsAll)) return 1;   
    read_object_pts(fname,coarseObjectPoints2);    
    interpolate_key_points(coarseObjectPoints2,objectPoints); //interpolate and append pts to objectPoints vector
    /* */
    
    /**/
    //image 3:
    fname = "image3_pretransformed_corners.csv";
    if(!read_image_pts(fname,imagePoints)) return 1;
    //read in points again, extracting just the 4 corners for coarse key points
    tempImagePoints.clear();
    read_image_pts(fname,tempImagePoints);
    
    //extract just the 4 corner key points:
    coarseImagePoints.push_back(tempImagePoints[0]); //aft/starboard
    coarseImagePoints.push_back(tempImagePoints[17]);
    coarseImagePoints.push_back(tempImagePoints[126]);
    coarseImagePoints.push_back(tempImagePoints[143]);    
    
    fname = "image3_key_pts_metric.csv";
    //coarseObjectPoints.clear();
    if(!read_object_pts(fname,coarseObjectPointsAll)) return 1;   
    read_object_pts(fname,coarseObjectPoints3);    
    interpolate_key_points(coarseObjectPoints3,objectPoints); //interpolate and append pts to objectPoints vector      
    
    //image 4
    /**/
    fname = "image4_pretransformed_corners.csv";
    if(!read_image_pts(fname,imagePoints)) return 1;
    //read in points again, extracting just the 4 corners for coarse key points
    tempImagePoints.clear();
    read_image_pts(fname,tempImagePoints);
    
    //extract just the 4 corner key points:
    coarseImagePoints.push_back(tempImagePoints[0]); //aft/starboard
    coarseImagePoints.push_back(tempImagePoints[17]);
    coarseImagePoints.push_back(tempImagePoints[126]);
    coarseImagePoints.push_back(tempImagePoints[143]);    
    
    
    fname = "image4_key_pts_metric.csv";
    //coarseObjectPoints.clear();
    if(!read_object_pts(fname,coarseObjectPointsAll)) return 1;   
    read_object_pts(fname,coarseObjectPoints4);        
    interpolate_key_points(coarseObjectPoints4,objectPoints); //interpolate and append pts to objectPoints vector       
/* */
    
      string interp_pts_file = "interp_pts.csv";
    

    save_to_csv(interp_pts_file,objectPoints);
     
     Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
     cameraMatrix.at<double>(0,0) = fx;
     cameraMatrix.at<double>(1,1) = fy;
     cameraMatrix.at<double>(0,2) = ci; //is this correct???
     cameraMatrix.at<double>(1,2) = cj;
     
     Mat distCoeffs;
     Mat rvec,tvec;
     
     //see: https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#ga549c2075fac14829ff4a58bc931c033d
     solvePnP(coarseObjectPointsAll,coarseImagePoints,cameraMatrix,distCoeffs,rvec,tvec);
     std::cout << "cameraMatrix : "<<endl << cameraMatrix << std::endl;

     // std::cout << "distCoeffs : " << distCoeffs << std::endl;

     std::cout << "Rotation vector : " << endl<< rvec << std::endl;

     std::cout << "Translation vector : " << endl<< tvec << std::endl;
     
     //compute reprojection error:
     
     projectPoints(coarseObjectPointsAll,rvec,tvec,cameraMatrix,distCoeffs,imagePointsReprojected);
     cout<<"image points: "<<endl<<coarseImagePoints<<endl;
     cout<<"reprojections: "<<endl<<imagePointsReprojected<<endl;
     //Mat pts_diff = imagePointsReprojected-imagePoints;
     int coarse_npts = coarseImagePoints.size();
     cout<<"coarse npts = "<<coarse_npts<<endl;

     
    double err = norm(imagePointsReprojected,coarseImagePoints);
     

     cout<<"rms err from solvePnP: "<<err/sqrt(coarse_npts)<<endl;
     //cout<<"err each pt: "<<endl;
     
     //cout<<"enter 1: ";
     //cin>>g_ans;
     
     cout<<"using all image points and interpolated object points: "<<endl;
     solvePnP(objectPoints,imagePoints,cameraMatrix,distCoeffs,rvec,tvec);
     std::cout << "cameraMatrix : "<<endl << cameraMatrix << std::endl;

     // std::cout << "distCoeffs : " << distCoeffs << std::endl;

     std::cout << "Rotation vector : " << endl<< rvec << std::endl;

     std::cout << "Translation vector : " << endl<< tvec << std::endl;
     
     //compute reprojection error:
     
     projectPoints(objectPoints,rvec,tvec,cameraMatrix,distCoeffs,imagePointsReprojected);
     //cout<<"image points: "<<endl<<imagePoints<<endl;
     //cout<<"reprojections: "<<endl<<imagePointsReprojected<<endl;
     //Mat pts_diff = imagePointsReprojected-imagePoints;
     int npts = imagePoints.size();
     cout<<"npts = "<<npts<<endl;

     
    err = norm(imagePointsReprojected,imagePoints);
     

     cout<<"rms err from solvePnP: "<<err/sqrt(npts)<<endl;
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
    //Eigen::Affine3d affine_cam_wrt_sys = get_hardcoded_affine_sys_wrt_cam();
    Eigen::Affine3d affine_cam_wrt_sys = get_hardcoded_affine_cam_wrt_sys(); 
    ROS_INFO("using affine_cam_wrt_RBIP: ");
    xformUtils.printAffine(affine_cam_wrt_sys);
    Eigen::Affine3d affine_sys_wrt_cam = affine_cam_wrt_sys.inverse();
    Eigen::Affine3d affine_cam_wrt_sys_perturbed,affine_correction_camera_frame_wrt_sys, affine_best;
    Eigen::Vector3d dtrans;
    //dtrans<<0.2,0,0;
    double rms_err;
    rms_err= attempt_fit_points(affine_sys_wrt_cam,imagePoints,objectPoints);
    ROS_WARN("rms_err init = %f",rms_err);

         double dx=0;
         double dy=0;
         double dz=0;
         double dphix=0;
         double dphiy=0;
         double dphiz=0;     
         double rms_err_min = 1000;
         
     //affine_cam_wrt_sys..translation()=affine_cam_wrt_sys..translation()+dtrans;

         
         for (dx=-3*g_dsearch_trans;dx<3*g_dsearch_trans;dx+=g_dsearch_trans) {
             ROS_INFO("dx = %f",dx);
             for (dy=-3*g_dsearch_trans;dy<3*g_dsearch_trans;dy+=g_dsearch_trans) {
                 for (dz=-3*g_dsearch_trans;dz<3*g_dsearch_trans;dz+=g_dsearch_trans) {
                     for (dphix=-3*g_dsearch_rot;dphix<3*g_dsearch_rot;dphix+=g_dsearch_rot) {
                         for (dphiy=-3*g_dsearch_rot;dphiy<3*g_dsearch_rot;dphiy+=g_dsearch_rot) {
                             for (dphiz=-3*g_dsearch_rot;dphiz<3*g_dsearch_rot;dphiz+=g_dsearch_rot) { //(int i=0;i<npts;i++)

         Eigen::Vector3d dtrans;
         //dtrans<<dx,0,0;

         dtrans<<dx,dy,dz;

         //cout<<"dtrans: "<<dtrans.transpose()<<" dphi: "<<dphix<<", "<<dphiy<<", "<<dphiz<<endl;
         //cout<<"dphi: "<<dphix<<", "<<dphiy<<", "<<dphiz<<endl;
         affine_correction_camera_frame_wrt_sys = compute_correction_transform(dx, dy, dz, dphix, dphiy,dphiz);        
         affine_cam_wrt_sys_perturbed= affine_correction_camera_frame_wrt_sys*affine_cam_wrt_sys;
         //affine_cam_wrt_sys_perturbed.translation()= affine_cam_wrt_sys_perturbed.translation()+dtrans;
         affine_sys_wrt_cam = affine_cam_wrt_sys_perturbed.inverse();
         rms_err = attempt_fit_points(affine_sys_wrt_cam,imagePoints,objectPoints);
         //    ROS_INFO("rms_err = %f",rms_err);


         if (rms_err<rms_err_min) {
             rms_err_min = rms_err;
             ROS_WARN("rms_err_min = %f",rms_err_min);
             cout<<"dtrans: "<<dtrans.transpose()<<endl;
             cout<<"dphi: "<<dphix<<", "<<dphiy<<", "<<dphiz<<endl;    
             affine_best=affine_cam_wrt_sys_perturbed;
             xformUtils.printAffine(affine_best);
         }
                        }
                      }
                     }
                 }
             } //dy loop
         }
         ROS_WARN("affine best: ");
         xformUtils.printAffine(affine_best);
         ROS_INFO("rms error: %f",rms_err_min);
         Eigen::Vector3d Origin;
         Eigen::Matrix3d Rot;
         Origin = affine_best.translation();
         cout<<endl;
         ROS_WARN("COPY/PASTE ME:");
         cout<<endl;
         cout<<"trans<<"<< Origin[0]<<", "<<Origin[1]<<", "<<Origin[2]<<";//"<<rms_err_min<<endl;
         Rot=affine_best.linear();
         cout<<"R<<"<< Rot(0,0)<<", "<<Rot(0,1)<<", "<<Rot(0,2)<<","<<endl;
         cout<<"    "<< Rot(1,0)<<", "<<Rot(1,1)<<", "<<Rot(1,2)<<","<<endl;
         cout<<"    "<< Rot(2,0)<<", "<<Rot(2,1)<<", "<<Rot(2,2)<<";"<<endl;
         cout<<endl;
         
         
         affine_sys_wrt_cam = affine_best.inverse();
         ROS_INFO("affine sys_wrt_cam: ");
          xformUtils.printAffine(affine_sys_wrt_cam);
         //cout<<"dtrans: "<<dtrans.transpose()<<endl;
         //cout<<"dphi: "<<dphix<<", "<<dphiy<<", "<<dphiz<<endl;       
    return 0;
}
