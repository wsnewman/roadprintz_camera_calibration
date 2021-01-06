//crude, manual approach to finding camera extrinsics;
//involves copy/paste of output back into this program to do manually guided convergence


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

Eigen::Affine3d get_hardcoded_affine_cam_wrt_sys(void) {
	Eigen::Affine3d hardcoded_affine_cam_wrt_sys;

	Eigen::Vector3d trans;
	Eigen::Matrix3d R;
	Eigen::Vector3d xvec_cam_wrt_sys,yvec_cam_wrt_sys,zvec_cam_wrt_sys;
    //v<< 2.81, 0,  2.87; //vector from origin of system_ref_frame to camera frame
   
    //R <<  -1, 0,  -0,   //orientation of camera frame w/rt system_ref_frame, optical axis points in neg z direction
    //           0, 1, 0,
    //           0, 0, -1;   

     //   trans<<  2.76796, 0.00720234,    2.88546; //4.5426 rms pixel error in reprojection based on ~1,000 points
        
      //  trans<<  2.76911, 0.00777921,    2.88495; //4.44
      //  trans<<  2.76797, 0.00830871,    2.88722;
      //  trans<<  2.76683, 0.00883616,    2.88949;//7.047 rms err
      //  trans<< 2.76569, 0.00936156,    2.89176; //6.314
      //  trans<< 2.76455, 0.00868492,    2.89402; //5.851
      //  trans<<2.76341, 0.00690042,    2.89628; //5.485
      //  trans<<2.76227, 0.00511434,    2.89854;//5.168
       // trans<<2.76287, 0.00332767,    2.89794; //4.892
       // trans<<2.76405, 0.00154181,    2.89699; //4.619384
       // trans<<  2.76523, -0.000596052,      2.89584; //4.346809
       // trans<< 2.76641, -0.00238018,     2.89469; //4.076635
       // trans<<2.76817, -0.00451633,     2.89298; //3.807012
       // trans<<2.76935, -0.00625178,     2.89182; //3.539035
       // trans<< 2.77053, -0.00818654,     2.89066; //3.272630
       // trans<<2.77171, -0.0101206,     2.8895; //3.008528
       // trans<<2.77289, -0.0118083,    2.88834; //2.745506
        //above is for image1 ONLY
       //try w/ inputs 1,2,5,6,8, 11, 13 
       //trans<< 2.78842, 0.00345516,    2.87124;  //5.53223
       //trans<<2.79042, 0.0128573,   2.86821;  //4.775859
       //trans<< 2.78981, 0.0128573,   2.86806  ;//4.502246
       // trans<<2.7912, 0.0127102,   2.86666  ;//4.358967
       // trans<<2.79391, 0.0107049,   2.86611 ; //4.333166
       // trans<<2.79507, 0.0104049,   2.86587 ; //4.328827
                        trans<<2.7955,  0.0109847,    2.8659 ;//4.328155
        //compare to previous: 2.76911, 0.00777921,   2.88495;
        
    //note: camera origin is about 2.885 meters high, and about 2.768m aft of rear wheels
    //   camera origin is nearly along vehicle centerline, but shifted about 7mm to starboard

    // R <<     0.0525021,  0.995724,   0.0759998,
    //          0.9976,    -0.055738,   0.0410976,
    //          0.0451579,  0.0736596, -0.996261;
   
     R<<  0.0483989,   0.996187,  0.0725811,
  0.997816, -0.0514923,    0.04137,
 0.0449498,  0.0704207,  -0.996505;
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
    
    fname = "image11_corners.csv";
    if(!read_image_pts(fname,imagePoints)) return 1;
    fname = "lidar_pts11_metric.csv";
    if(!read_object_pts(fname,objectPoints)) return 1;   

    fname = "image13_corners.csv";
    if(!read_image_pts(fname,imagePoints)) return 1;
    fname = "lidar_pts13_metric.csv";
    if(!read_object_pts(fname,objectPoints)) return 1;   
  /* */
    

     
     Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
     cameraMatrix.at<double>(0,0) = fx;
     cameraMatrix.at<double>(1,1) = fy;
     cameraMatrix.at<double>(0,2) = ci; //is this correct???
     cameraMatrix.at<double>(1,2) = cj;
     
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
     //cout<<"image points: "<<endl<<imagePoints<<endl;
     //cout<<"reprojections: "<<endl<<imagePointsReprojected<<endl;
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
    //Eigen::Affine3d affine_cam_wrt_sys = get_hardcoded_affine_sys_wrt_cam();
    Eigen::Affine3d affine_cam_wrt_sys = get_hardcoded_affine_cam_wrt_sys(); 
    Eigen::Affine3d affine_sys_wrt_cam = affine_cam_wrt_sys.inverse();
    Eigen::Affine3d affine_cam_wrt_sys_perturbed,affine_correction_camera_frame_wrt_sys, affine_best;
    Eigen::Vector3d dtrans;
    //dtrans<<0.2,0,0;
     attempt_fit_points(affine_sys_wrt_cam,imagePoints,objectPoints);
     
         double dx=0;
         double dy=0;
         double dz=0;
         double dphix=0;
         double dphiy=0;
         double dphiz=0;     
         double rms_err_min = 1000;
         double rms_err;
     //affine_cam_wrt_sys..translation()=affine_cam_wrt_sys..translation()+dtrans;
        //choose search for about 2min, i.e. test center and +/- three samples either side
         double dsearch = 0.00015;
         
         for (dx=-3*dsearch;dx<3*dsearch;dx+=dsearch) {
             for (dy=-3*dsearch;dy<3*dsearch;dy+=dsearch) {
                 for (dz=-3*dsearch;dz<3*dsearch;dz+=dsearch) {
                     for (dphix=-3*dsearch;dphix<3*dsearch;dphix+=dsearch) {
                         for (dphiy=-3*dsearch;dphiy<3*dsearch;dphiy+=dsearch) {
                             for (dphiz=-3*dsearch;dphiz<3*dsearch;dphiz+=dsearch) { //(int i=0;i<npts;i++)

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
             }
         }
         ROS_INFO("affine best: ");
         xformUtils.printAffine(affine_best);
         ROS_INFO("rms error: %f",rms_err_min);
         affine_sys_wrt_cam = affine_best.inverse();
         ROS_INFO("affine sys_wrt_cam: ");
          xformUtils.printAffine(affine_sys_wrt_cam);
         //cout<<"dtrans: "<<dtrans.transpose()<<endl;
         //cout<<"dphi: "<<dphix<<", "<<dphiy<<", "<<dphiz<<endl;       
    return 0;
}
