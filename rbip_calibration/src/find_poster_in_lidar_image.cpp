//program to find RoadPrintz checkerboard poster from image
//SPECIALIZED for 18 keypoints x 8 keypoints


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


//static const std::string OPENCV_WINDOW = "ImageWindow";
using namespace cv;
using namespace std;
using namespace Eigen;
int g_ans;

static double EXPECTED_NPTS=144; //customized for 18x8 poster, which has 144 square-square intersections
int SCALE_DOWN_FACTOR=4;

static double SQUARE_SIZE_METRIC=0.1007; //set known size of squares in poster; MATCH THIS TO find_poster_v2.cpp


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

//the following fncs are copied from find_poster_v2.cpp
//sum of lengths of centered points is useful for scaling
double sum_norms(vector<Eigen::Vector2d>pts) {
    int npts = pts.size();
    //compute the sum of norms of centered pts
    double sum_of_norms=0;
    for (int i=0;i<npts;i++) {
        sum_of_norms+=pts[i].norm();
    }   
    return sum_of_norms;
} 

void make_ideal_grid(vector<Eigen::Vector2d> &ideal_grid_pts,double &sum_of_norms) {
    ideal_grid_pts.clear();
    double sum_uvals =0;
    double sum_vvals =0;
    Eigen::Vector2d pt;
    for (int col=0;col<8;col++) {
        double u = -col*KPIX*SQUARE_SIZE_METRIC/SCALE_DOWN_FACTOR;
        pt[0]=u;
        for (int row=0;row<18;row++) {
            double v = row*KPIX*SQUARE_SIZE_METRIC/SCALE_DOWN_FACTOR;
            pt[1]=v;
            ideal_grid_pts.push_back(pt);
            sum_uvals+=u;
            sum_vvals+=v;
        }
    }  
    int npts = ideal_grid_pts.size();
    double u_mean = sum_uvals/npts;
    double v_mean = sum_vvals/npts;
    for (int i=0;i<npts;i++) {
        pt = ideal_grid_pts[i];
        pt[0]-= u_mean;
        pt[1]-= v_mean;
        ideal_grid_pts[i]=pt;
    }
    //compute the sum of norms of centered pts
    sum_of_norms=sum_norms(ideal_grid_pts);

}

void cv_corners_to_eigen(vector<Point2f> corners, vector<Eigen::Vector2d> &eigen_pts) {
    eigen_pts.clear();
    Eigen::Vector2d pt;
    int npts = corners.size();
    for (int i=0;i<npts;i++) {
        pt[0] = corners[i].x;
        pt[1] = corners[i].y;
        eigen_pts.push_back(pt);
    }
}

Eigen::Vector2d find_centroid(vector<Eigen::Vector2d> eigen_pts) {
    int npts = eigen_pts.size();
    double x_sum=0.0;
    double y_sum=0.0;
    for (int i=0;i<npts;i++) {
        x_sum+=(eigen_pts[i])[0];
        y_sum+=(eigen_pts[i])[1];
    }
    Eigen::Vector2d centroid;
    centroid[0]=x_sum/npts;
    centroid[1]=y_sum/npts;
    return centroid;
}

void subtract_centroid(vector<Eigen::Vector2d> eigen_pts,vector<Eigen::Vector2d> &eigen_pts_shifted) {
    int npts = eigen_pts.size();
    eigen_pts_shifted.clear();
    
    Eigen::Vector2d centroid,pt;
    centroid = find_centroid(eigen_pts);
    for (int i=0;i<npts;i++) {
        pt = eigen_pts[i];
        pt-=centroid;
        eigen_pts_shifted.push_back(pt);
    }
}
Eigen::MatrixXd convert_vec_of_pts_to_MatrixXd(vector<Eigen::Vector2d> pts_vec) {
    int npts = pts_vec.size();
    MatrixXd m(2,npts);  //or m.resize(2,npts);
    for (int i=0;i<npts;i++) {
        m.col(i) = pts_vec[i];
    }
    return m;
}

//2-D points are put in matrices as columns of points
double mat_pts_err(Eigen::MatrixXd ideal_grid_mat_rot,Eigen::MatrixXd feature_pts_shifted_mat){
    double rms_err=0;
    int npts = ideal_grid_mat_rot.cols();
    int feature_npts = feature_pts_shifted_mat.cols();
    ROS_INFO("npts of grid= %d; npts of features = %d",npts,feature_npts);
    Eigen::MatrixXd pts_diff_mat = ideal_grid_mat_rot-feature_pts_shifted_mat;
    //int npts = pts_diff_mat.cols();
    Eigen::Matrix2d sqd_err_mat = pts_diff_mat*pts_diff_mat.transpose();
    cout<<"sqd_err_mat: "<<endl<<sqd_err_mat<<endl;
    rms_err = sqrt((sqd_err_mat(0,0)+sqd_err_mat(1,1))/npts);
    return rms_err;
}

   
double find_grid_fit(vector<Eigen::Vector2d>image_feature_pts,vector<Eigen::Vector2d>ideal_grid_pts,
          vector<Eigen::Vector2d> &fit_grid_pts, vector<Eigen::Vector2d> &fit_grid_pts_upscaled) {
    Eigen::Vector2d centroid_grid,centroid_features, test_centroid_features;
    vector<Eigen::Vector2d> feature_pts_shifted;
    centroid_grid = find_centroid(ideal_grid_pts);
    centroid_features = find_centroid(image_feature_pts);
    ROS_INFO_STREAM("centroid_grid: "<<centroid_grid.transpose()<<endl);
    ROS_INFO_STREAM("centroid_features: "<<centroid_features.transpose()<<endl);
    subtract_centroid(image_feature_pts,feature_pts_shifted);
    test_centroid_features = find_centroid(feature_pts_shifted); //TEST: should result in zeros
    ROS_INFO_STREAM("centroid_features after offset computation: "<<test_centroid_features.transpose()<<endl);
    double sum_norms_features = sum_norms(feature_pts_shifted);
    double sum_norms_template = sum_norms(ideal_grid_pts);
    double scale = sum_norms_features/sum_norms_template;
    ROS_INFO("sum_norms_features = %f; sum_norms_template = %f; ratio = %f",sum_norms_features,sum_norms_template,scale);
    
       // get rotation
    Eigen::MatrixXd ideal_grid_mat,feature_pts_shifted_mat;
    ideal_grid_mat = convert_vec_of_pts_to_MatrixXd(ideal_grid_pts);
    feature_pts_shifted_mat = convert_vec_of_pts_to_MatrixXd(feature_pts_shifted);
    auto covMat = ideal_grid_mat * feature_pts_shifted_mat.transpose();
    cout<<"covMat"<<endl<<covMat<<endl;
    auto svd = covMat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    auto rot = svd.matrixV() * svd.matrixU().transpose(); 
    cout<<"rot = "<<endl<<rot<<endl;
    //convert to a rotation angle:
    double theta = atan2(rot(1,1),rot(0,0));
    ROS_WARN("corresponding rotation angle = %f",theta);
    ROS_WARN_STREAM("and offset: "<<centroid_features.transpose()<<endl);
    
    
    //double theta_z=0;
    Eigen::Matrix2d Rotz = rot; //renaming rotation matrix from above
    //Vector3d v(1,2,3);
    //Eigen::Vector2d col1(cos(theta_z),-sin(theta_z));
    //Eigen::Vector2d col2(sin(theta_z),cos(theta_z));

    //Rotz.col(0) = col1; //=cos(rot);
    //Rotz.col(1) = col2; //=cos(rot);

    Eigen::MatrixXd ideal_grid_mat_rot;
    cout<<"rotating ideal grid: "<<endl;
    ideal_grid_mat_rot=Rotz*ideal_grid_mat;
    cout<<"rotated ideal grid; calling mat_pts_err... "<<endl;
    
    double rms_err = mat_pts_err(ideal_grid_mat_rot,feature_pts_shifted_mat);
    ROS_INFO("rms_err of fit = %f",rms_err);
    double rms_err_scaled = mat_pts_err(ideal_grid_mat_rot*scale,feature_pts_shifted_mat);
    ROS_INFO("rms_err of fit for scaled grid= %f",rms_err_scaled);
    
    //although scaled grid can have lower RMS error, coerce it to be known dimensions, to match w/ LIDAR pts
    
    //populate vector<Eigen::Vector2d> &fit_grid_pts
    fit_grid_pts.clear();
    int npts = ideal_grid_pts.size();
    Eigen::Vector2d pt;
    for (int i=0;i<npts;i++) {
        pt = ideal_grid_mat_rot.col(i);
        pt = pt+centroid_features;
        fit_grid_pts.push_back(pt);
    }
    fit_grid_pts_upscaled.clear();
    for (int i=0;i<npts;i++) {
        pt = ideal_grid_mat_rot.col(i)*SCALE_DOWN_FACTOR;
        pt = pt+centroid_features*SCALE_DOWN_FACTOR;
        fit_grid_pts_upscaled.push_back(pt);
    }    
    
    
    //test the resulting best-fit grid pts:
    Eigen::MatrixXd fit_grid_mat,feature_pts_mat;
    fit_grid_mat = convert_vec_of_pts_to_MatrixXd(fit_grid_pts);
    feature_pts_mat = convert_vec_of_pts_to_MatrixXd(image_feature_pts);
    
    double test_err = mat_pts_err(fit_grid_mat,feature_pts_mat);
    ROS_INFO("test of shifted grid pts: fit err = %f",test_err);
    return rms_err; //-1; //rms_err<0--> failure
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

    //cout<<"converting to grayscale"<<endl;
    //cvtColor(src, src_gray, CV_BGR2GRAY);
    
    //cv::resize(img, img_dst, cv::Size(640, 480), 0, 0, cv::INTER_AREA);
    //I had trouble fitting chessboard to full-sized image; try scaling down first:
    resize(src,src_gray,cv::Size(src.cols/SCALE_DOWN_FACTOR, src.rows/SCALE_DOWN_FACTOR), 0, 0, cv::INTER_AREA);
    ROS_INFO("resized to %d by %d",src_gray.cols,src_gray.rows);
    vector<Point2f> corners;
    vector<Point2f> ordered_4_keypts;
    Size patternsize(18,8); //interior number of corners
    cout<<"calling findChessboardCorners"<<endl;
    bool patternfound = findChessboardCorners( src_gray, patternsize, corners); //, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
    if (!patternfound) {
      cout<<"did not find poster"<<endl;
      return 1;
    }  
    
    int n_corners = corners.size();
    if (n_corners!=EXPECTED_NPTS) {
        ROS_WARN("something wrong; pts mismatch; n_corners found = %d",n_corners);
    }
    cout<<"found "<<n_corners<<" corners"<<endl;
    //note: x,y output shows x must be along width, y is along height
    //for (int i=0;i<n_corners;i++) {
       //cout<<"corner: "<<corners[i].x<<", "<<corners[i].y<<endl;
    //}

    //fit ideal grid to identified key points:
    
    vector<Eigen::Vector2d> ideal_grid_pts,image_feature_pts,fit_grid_pts,fit_grid_pts_upscaled;
    //maybe ideal gridpoints does not need to be created for different rasters of image pts;
    //start w/ pts from upper-left, and raster down, assuming portrait mode
    // hopefully, ideal rotation will discover large rotations, if necessary
    double sum_norms;
    make_ideal_grid(ideal_grid_pts,sum_norms);
    ROS_INFO("ideal grid sum of norms = %f",sum_norms);
    //convert image feature points from OpenCV type to Eigen type
    cv_corners_to_eigen(corners, image_feature_pts);
    //double find_grid_fit(vector<Eigen::Vector2d>image_feature_pts,vector<Eigen::Vector2d>ideal_grid_pts,
    //      vector<Eigen::Vector2d> &fit_grid_pts, vector<Eigen::Vector2d> &fit_grid_pts_upscaled)
    double rms_err = find_grid_fit(image_feature_pts,ideal_grid_pts,fit_grid_pts,fit_grid_pts_upscaled);
    if (rms_err<0) {
        ROS_WARN("COULD NOT FIT GRID POINTS TO POSTER FEATURES");
        return 1;
    }
    ROS_INFO("fit ideal grid to image features w/ rms_err = %f",rms_err);

    
    //subPix was not working; don't know why; skip it
    /*
    cout<<"calling cornerSubPix"<<endl;
    cornerSubPix(src_gray, corners, Size(11, 11), Size(-1, -1),
                     TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    cout<<"done with cornerSubPix; enter 1:";
    cin>>g_ans; */
    
    //at this point, have pts in fit_grid_pts, but these are 1/4 scale of the original image
    //convert these back to the original scale and save the points
    
    
    
    
    ofstream keypoints_file,corners_file,gridpts_file;
    string corners_filename =fname_root+"_corners.csv";
    string keypoints_filename =fname_root+"_keypoints.csv";
    string grid_filename =fname_root+"_gridpoints.csv";
    
        gridpts_file.open(grid_filename.c_str(), ios::out | ios::trunc);
        int n_gridpts = fit_grid_pts.size();
        Eigen::Vector2d grid_pt;
    for (int i=0;i<n_gridpts;i++) {
            grid_pt = fit_grid_pts_upscaled[i];
            //cout<<"template corner: "<<grid_pt[0]<<", "<<grid_pt[1]<<endl;
                   
            gridpts_file << grid_pt[0]<<", "<<grid_pt[1]<<endl;          
        }
        gridpts_file.close();    
        
    //keypoints_filename
    
        keypoints_file.open(keypoints_filename.c_str(), ios::out | ios::trunc);
        
    for (int i=0;i<n_corners;i++) {
            //cout<<"corner: "<<corners[i].x<<", "<<corners[i].y<<endl;
                   
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
    drawChessboardCorners(src_gray, patternsize, Mat(corners), patternfound);

    //save image:
    string processed_fname = "processed_"+fname;
    imwrite(processed_fname,src_gray);

    cout<<"calling imshow"<<endl;
    //Mat src_smaller;
    //resize(src, src_smaller, Size(src.cols/2,src.rows/2));
    imshow("result",src_gray);
 
    /// Wait until user exits the program by pressing a key
    cout<<"calling waitKey(0)"<<endl;
    waitKey(0);
    cout<<"done; returning 0"<<endl;
    return 0;
}
