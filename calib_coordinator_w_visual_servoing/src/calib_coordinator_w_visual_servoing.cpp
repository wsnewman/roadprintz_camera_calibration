//calib_coordinator_w_visual_servoing.cpp
//wsn, 2/26/21
//run this together with:
//roslaunch realsense2_camera rp_rs_camera.launch 
//rosrun test_moves l515_move_service
//rosrun rbip_calibration find_poster_in_lidar_image_10x7_service
//rosrun pcl_utils make_image_from_cloud_service

/*    
 *move robot to a hard-coded nominal pose for visual-servo testing
 *get images from L515
 *find key point in L515 image
 *compute incremental motions to move L515 over key point
 *prompt user for incremental actions
 *send commands to l515_move_service
 *offer option of rotations
 data acquisition will be manual
 */


//send commands for x,y, and z and return to camera pose w/ alt topic pub 
#include <ros/ros.h>
#include <test_moves/TestMoveSrv.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

//#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <single_corner/single_corner.h>

#include <sensor_msgs/image_encodings.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>
#include <rbip_calibration/CamPosterCalibTestSrv.h>
#include <pcl_generator/MapperSrv.h>
#include <std_srvs/Trigger.h>


using namespace std;
using namespace cv;

typedef vector <double> record_t;
typedef vector <record_t> data_t;

string g_rbif_pts_filename, g_rect_image_fname, g_virtual_image_fname;
string g_virtual_root_name, g_rect_root_name;
string g_robot_pts_filename;
string g_fname_root = "image";
string g_fname_root_w_index;
string g_data_dir;
int g_fname_index = 0;
vector<Point2d> g_sample_pts;
vector<int> g_sample_pt_indices;

bool g_do_more_posters = true;
bool g_all_is_well = true;


int g_ans = 0;

string g_intel_image_topic = "/camera/color/image_raw"; //set input topic name
string g_arm_cam_rect_image_topic = "/camera_pretransformed/image_rect_color"; //set input topic name
string g_arm_cam_virt_image_topic = "/virtual_camera/image_rect_color"; //set input topic name

Mat g_src, g_src2, g_src3;
bool g_got_new_image = false;
bool g_got_new_image2 = false;
bool g_got_new_image3 = false;

const double L515_INSPECTION_HEIGHT = 0.2; // desired height of toolflange-mounted camera
const double L515_THETAZ = 2.84;
const double L515_ROT_CTR_UC = 327.9;
const double L515_ROT_CTR_VC = 251.4;
const double L515_PIX_PER_METER = 4100;

//OpenCvUtils g_open_cv_utils(cam_topic_name);

//const int N_SCAN_SAVES = 3;

const double X_MIN = 2; //1.75; //map runs from this min value...
const double X_MAX = 2; //5.5; //map out this far from rear wheels. Increased due to weird behavior in interpolation towards the limits
const double Y_MIN = 0; //3.0; //map this far to left
const double Y_MAX = 0; //3.0; //and this far to right
const double Z_MIN = 0.9;
const double Z_MAX = 1.4;

const double DX = 0.5;
const double DY = 0.5;
const double DZ = 0.1;

const double EPS = 0.01; //for tolerances
const double VISUAL_SERVO_CONVERGENCE_TOL = 0.0002;
const int VISUAL_SERVO_MAX_TRIES = 4;

string g_dir_name;
vector<string> g_image_names;

vector<double> g_scan_vec_of_dbls;
bool g_got_lidar_data = false;
//careful near robot base; interferes with self

istream& operator>>(istream& ins, record_t& record) {
    // make sure that the returned record contains only the stuff we read now
    record.clear();

    // read the entire line into a string (a CSV record is terminated by a newline)
    string line;
    getline(ins, line);
    cout << "getline length: " << line.size() << endl;


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

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    g_src = cv_ptr->image;
    g_got_new_image = true; //note that new image has been acquired
}

void imageCallback2(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    g_src2 = cv_ptr->image;
    g_got_new_image2 = true; //note that new image has been acquired
}

void imageCallback3(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    g_src3 = cv_ptr->image;
    g_got_new_image3 = true; //note that new image has been acquired
}

string gen_fname(int i, int j, int k, int r) {
    stringstream ss;
    ss << g_dir_name;
    ss << "image_" << k << "_" << j << "_" << i << "_" << r << ".png";
    string str = ss.str();

}

//    g_robot_pts_filename=g_data_dir+g_fname_root<<g_fname_index+"_robot_keypoints.csv";

void gen_output_fnames(int image_index) {
    stringstream ss, ss2, ss3, ss4,ss5,ss6;
    ss4 << g_fname_root << g_fname_index << "_";
    g_fname_root_w_index = ss4.str();
    
    ss << g_data_dir + g_fname_root_w_index + "robot_keypoints.csv";

    //ss << g_data_dir + g_fname_root << g_fname_index + "_robot_keypoints.csv";
    g_robot_pts_filename = ss.str();
    ss2 << g_data_dir + g_fname_root_w_index + "rect.png";
    g_rect_image_fname = ss2.str();
    ss5 << g_data_dir + g_fname_root_w_index + "rect";
    g_rect_root_name = ss5.str();
    
    ROS_INFO_STREAM("g_rect_image_fname: "<<g_rect_image_fname<<endl);
    ss3 << g_data_dir + g_fname_root_w_index + "virtual.png";
    g_virtual_image_fname = ss3.str();
    
    ss6 << g_data_dir + g_fname_root_w_index + "virtual";
    g_virtual_root_name = ss6.str();
    
}

void set_sample_poses(vector<Eigen::VectorXd> &sample_poses, vector<string> &fnames) {
    sample_poses.clear();
    Eigen::VectorXd pose;
    pose.resize(4);
    //double x,y,z;
    int i, j, k;
    int i_max = round((X_MAX - X_MIN) / DX);
    int j_max = round((Y_MAX - Y_MIN) / DY);
    int k_max = round((Z_MAX - Z_MIN) / DZ);

    //outfile.open(pose_info);
    //pose[2] = SCAN_HEIGHT;
    //fill and stuff sample_poses with vectors to visit
    for (k = 0; k <= k_max; k++) {
        for (j = 0; j <= j_max; j++) {
            for (i = 0; i <= i_max; i++) {
                for (int r = 0; r < 4; r++) {
                    pose[0] = X_MIN + i*DX;
                    pose[1] = Y_MIN + j*DY;
                    pose[2] = Z_MIN + k*DZ;
                    pose[3] = r * M_PI / 2.0;
                    sample_poses.push_back(pose);
                    fnames.push_back(gen_fname(i, j, k, r));
                }
            }
        }
    }
}



//perhaps skip this and just assume commanded pose has been achieved

void get_robot_pose(Eigen::Vector3d &measured_robot_pose) {
    ROS_INFO("dummy fnc to get robot pose");

}

bool save_pose_and_scan_data(string result_file, Eigen::Vector3d pose, vector<double> scan_vec_of_dbls) {
    ROS_INFO("dummy fnc to write data acquisition to file");
    ofstream outfile;
    //outfile.open("test.txt", std::ios_base::app); // append instead of overwrite
    //outfile << "Data"; 
    outfile.open(result_file, std::ios_base::app); // append instead of overwrite
    outfile << pose[0] << ", " << pose[1] << ", " << pose[2] << ", ";
    int npts = scan_vec_of_dbls.size();
    for (int i = 0; i < npts - 1; i++) {
        outfile << scan_vec_of_dbls[i] << ", ";
    }
    outfile << scan_vec_of_dbls[npts - 1] << endl;

    outfile.close();

}

void get_scan_data() { //should assure a fresh scan; maybe also average the scans; put data in g_scan_vec_of_dbls
    ROS_INFO("dummy fnc to get scan info");
    g_scan_vec_of_dbls.clear();
    //DUMMY...
    /*for (int i=0;i<7;i++) {
        g_scan_vec_of_dbls.push_back((double)i);
    }*/
    // really, want to do this: 

    g_got_lidar_data = false;
    while (!g_got_lidar_data) {
        ros::spinOnce();
        ROS_INFO("waiting for scan data");
        ros::Duration(0.5).sleep();
    }


}



//this fnc is intended to read files estimated_rbif_keypoints.csv,
// which contain: checkerboard point index, x_val, y_val (in RBIP frame)

bool read_sample_pts(std::string fname, vector<Point2d> &sample_pts, vector<int> &sample_pt_indices) {

    ifstream infile(fname.c_str());
    if (!infile) { //The file couldn't be opened.
        cerr << "Error: file " << fname.c_str() << " could not be opened" << endl;
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
    cout << "CSV file " << fname << " contains " << data.size() << " records.\n";
    int nrecords = (int) data.size();
    sample_pts.clear();
    sample_pt_indices.clear();
    //image_pts.clear(); nope: append points
    for (int irecord = 0; irecord < nrecords; irecord++) {
        pt.x = data[irecord][1]; //data points are in (u,v) order, i.e. (col, row), per OpenCV convention
        pt.y = data[irecord][2];
        sample_pts.push_back(pt);
        sample_pt_indices.push_back(data[irecord][0]);
    }

    return true;
}

/*
bool get_rbip_est_key_points() {
    //vector<Point2d> &sample_pts,vector<int> &sample_pt_indices
    vector<Point2d> imagePoints,imagePointsReprojected;
    vector<Point3d> objectPoints;
    Point2d imagePoint;
    Point3d objectPoint;

   //bool read_sample_pts(std::string fname,vector<Point2d> &sample_pts,vector<int> &sample_pt_indices) {
    
    
    string fname = g_data_dir+"estimated_rbif_keypoints.csv";
    if(!read_sample_pts(fname,imagePoints)) return 1;
    fname = "lidar_pts1_metric.csv";
    if(!read_object_pts(fname,objectPoints)) return 1;   
}*/

/*
void processed_lidar_CB(const roadprintz_msgs::VecOfDoubles& vec_of_dbls_msg){
    if (!g_got_lidar_data) {
        //vec_of_dbls.dbl_vec
        int npts = vec_of_dbls_msg.dbl_vec.size();
        g_scan_vec_of_dbls.clear();
        g_scan_vec_of_dbls = vec_of_dbls_msg.dbl_vec; //this should work
        /* else do this...
        for (int i=0;i<npts;i++) {
            g_scan_vec_of_dbls.push_back(vec_of_dbls_msg.dbl_vec[i]);
        }
        
        g_got_lidar_data=true;
    }
}
 */

// move correction: ONLY works at L515_THETAZ = 2.84; and height L515_INSPECTION_HEIGHT = 0.2;
// specify fiducial, (u_fid,v_fid), in tp
// compute motion correction: (dx,dy) = [(v_fid-vc), (u_fid-uc)]/4100

bool compute_move_to_feature_center(cv::Point2d tp, double &dx, double &dy) {
    double u_fid = tp.x;
    double v_fid = tp.y;
    dx = (v_fid - L515_ROT_CTR_VC) / L515_PIX_PER_METER;
    dy = (u_fid - L515_ROT_CTR_UC) / L515_PIX_PER_METER;
    return true; //should have some error handling
}

void debug_print_sample_points() {
    Point2d pt;

    int nsamps = g_sample_pts.size();
    int index_samp;
    ROS_INFO("sample points read from file: ");
    for (int isamp = 0; isamp < nsamps; isamp++) {
        pt = g_sample_pts[isamp];
        index_samp = g_sample_pt_indices[isamp];
        ROS_INFO("%d, %f, %f", index_samp, pt.x, pt.y);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "l515_surface_scan_coordinator");
    ros::NodeHandle nh;
    //string result_file = "l515_scan_data_file.csv";
    //vector<Eigen::Vector3d> poses;
    //vector<vector<double>> scans;
    //poses.clear();
    //scans.clear();
    //roadprintz_msgs::VecOfDoubles vec_of_dbls;
    //vec_of_dbls.dbl_vec.resize(7); //make room for centroid and normal
    //ros::Publisher vec_publisher = nh.advertise<roadprintz_msgs::VecOfDoubles>("lidar_patch", 1);
    //ros::Subscriber processed_lidar_sub = nh.subscribe("lidar_patch",1,processed_lidar_CB);

    //prepare to use these services:
    ros::ServiceClient l515_moves_client = nh.serviceClient<test_moves::TestMoveSrv>("l515_moves_service");
    ros::ServiceClient mapper_client = nh.serviceClient<pcl_generator::MapperSrv>("surface_map_service");
    ros::ServiceClient find_poster_client = nh.serviceClient<rbip_calibration::CamPosterCalibTestSrv>("find_poster_service");
    ros::ServiceClient find_poster_in_image_client = nh.serviceClient<rbip_calibration::CamPosterCalibTestSrv>("find_poster_in_image_service");

    ros::ServiceClient lidar_to_image_client = nh.serviceClient<std_srvs::Trigger>("lidar_to_image_service");
//
    //instantiate messages for the services
    test_moves::TestMoveSrv move_srv;
    rbip_calibration::CamPosterCalibTestSrv find_poster_srv;
    rbip_calibration::CamPosterCalibTestSrv find_poster_in_image_srv;   
    pcl_generator::MapperSrv mapper_srv;    
    std_srvs::Trigger lidar_to_image_srv;

    string ros_ws_path = getenv("ROS_WORKSPACE");
    g_data_dir = ros_ws_path + "/calibration_temp_files/";
    g_rbif_pts_filename = ros_ws_path + "/calibration_temp_files/estimated_rbif_keypoints.csv";
    gen_output_fnames(g_fname_index);




    move_srv.request.z_rot_angle_increment.resize(1);
    move_srv.request.z_rot_angle_increment[0] = L515_THETAZ;
    move_srv.request.z_wrt_RBIP = L515_INSPECTION_HEIGHT;

    move_srv.request.rtn_to_camera_pose = false; //first call will NOT be return to camera pose

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(g_intel_image_topic.c_str(), 1, imageCallback);
    image_transport::ImageTransport it2(nh);
    image_transport::Subscriber sub2 = it2.subscribe(g_arm_cam_rect_image_topic.c_str(), 1, imageCallback2);
    image_transport::ImageTransport it3(nh);
    image_transport::Subscriber sub3 = it3.subscribe(g_arm_cam_virt_image_topic.c_str(), 1, imageCallback3);



    //test the cameras:

    ROS_INFO("testing the Intel camera...");

    g_got_new_image = false;
    while (!g_got_new_image) {
        ROS_INFO("waiting for image...");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    ROS_INFO_STREAM("got an image; should be able to view on camera topic " << g_intel_image_topic << endl);
    
    g_got_new_image2 = false;
    while (!g_got_new_image2) {
        ROS_INFO("waiting for arm camera rectified image ...");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    ROS_INFO_STREAM("got an image; should be able to view on camera topic " << g_arm_cam_rect_image_topic << endl);

    g_got_new_image3 = false;
    while (!g_got_new_image3) {
        ROS_INFO("waiting for arm camera virtual image...");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    ROS_INFO_STREAM("got an image; should be able to view on camera topic " << g_arm_cam_virt_image_topic << endl);

    ROS_INFO("camera topics are OK");
    

    //START THE MAIN LOOP
    while (g_do_more_posters) { //should repeat until user says enough
        
        ROS_INFO("poster capture number %d",g_fname_index);
        gen_output_fnames(g_fname_index); //update all filenames using current index
        ofstream robot_points_file;

            ROS_WARN("moving to camera pose");
            move_srv.request.rtn_to_camera_pose = true;
            l515_moves_client.call(move_srv);
            
        /*  *prompt to get lidar scan */
        ROS_WARN("MAKE SURE ROBOT IS IN CAMERA POSE");
        ROS_WARN("hit E-stop, then place poster");
        ROS_WARN("then re-enable the robot to start data acquisition");

        cout << "paused; enter 1 to capture poster image from camera pose: ";
        cin>>g_ans;
    g_got_new_image2 = false;
    while (!g_got_new_image2) {
        ROS_INFO("waiting for arm camera rectified image ...");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    ROS_INFO_STREAM("got an image; should be able to view on camera topic " << g_arm_cam_rect_image_topic << endl);
    ROS_INFO_STREAM("saving to: "<<g_rect_image_fname<<endl);
    imwrite(g_rect_image_fname,g_src2);
    
    g_got_new_image3 = false;
    while (!g_got_new_image3) {
        ROS_INFO("waiting for arm camera virtual image...");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    ROS_INFO_STREAM("got an image; should be able to view on camera topic " << g_arm_cam_virt_image_topic << endl);
    ROS_INFO_STREAM("saving to: "<<g_virtual_image_fname<<endl);

    imwrite(g_virtual_image_fname,g_src3);
    g_all_is_well=true;
    //attempt to find checkerboard in rectified, pre-transformed image
    ROS_INFO(" attempting to find checkerboard in pre-transformed, rectified image");
    
    find_poster_in_image_srv.request.fname = g_virtual_root_name.c_str();
    find_poster_in_image_client.call(find_poster_in_image_srv);
    if (!find_poster_in_image_srv.response.success) {
                ROS_WARN("was not able to find checkerboard pattern in image; move poster and try again");
                g_all_is_well = false;
                //g_fname_index--; //re-use previous filename index
    }    
    
    ROS_INFO(" attempting to find checkerboard in virtual image");   
    find_poster_in_image_srv.request.fname = g_rect_root_name.c_str();
    find_poster_in_image_client.call(find_poster_in_image_srv);
    if (!find_poster_in_image_srv.response.success) {
                ROS_WARN("was not able to find checkerboard pattern in image; move poster and try again");
                g_all_is_well = false;
                //g_fname_index--; //re-use previous filename index
    }             
    
           
        cout << "paused; enter 1 to start LIDAR sweep; enter 2 to skip scan; enter 0 to quit: ";
        cin>>g_ans;
        if (g_ans==0) {
            return 0;
        }
        if (g_ans == 1) {

            mapper_srv.request.request_code = pcl_generator::MapperSrvRequest::LOG_NEW_SCAN;
            //
            ROS_INFO("Requesting forward LIDAR scan");

            mapper_client.call(mapper_srv);
            bool response_code = mapper_srv.response.response_code;
            ROS_INFO("mapper service returned %d", response_code);
        } 
        
        else if (g_ans!=2) {
            ROS_WARN("quitting");
            g_do_more_posters = false;
            g_all_is_well = false; //do not do any of the rest of the steps
            return 0;
            //umm...not sure I want to do the following:
            //g_fname_index--; //re-use previous filename index
        }
        //convert LIDAR scan to an image
        if (g_all_is_well) {
            ROS_INFO("converting file to image");
            lidar_to_image_client.call(lidar_to_image_srv);

        //find poster:


            find_poster_srv.request.fname = g_fname_root_w_index.c_str();
            find_poster_client.call(find_poster_srv);
            if (!find_poster_srv.response.success) {
                ROS_WARN("was not able to find checkerboard pattern from LIDAR sweep; move poster and try again");
                g_all_is_well = false;
                //g_fname_index--; //re-use previous filename index
            }            
            
        }





        /*
         *use services to see if can find checkerboard in lidar scan
         *if so, compute if target poses are reachable
         if so, 
        increment the image_num; take snapshot of image; 
        try to fit checkerboard to image
        if successful: 
          rename the LIDAR data(?) or lidar image (?)
          save images rectified as well as virtual (though latter will change w/ new calibration)
          save keypoints of image to csv file w/ imagename
     
          sequence through moves in estimated_rbif_keypoints.csv
          start visual servoing...until convergence success/failure
          if success:
            record: index_number, robot_x, robot_y (from tf_listener?  from fk?) in file: imagename_robot_points.csv
         *rtn to camera pose
         *prompt user: 0 to quit, 1 to acquire new data
         */
        //step 1: do a lidar scan
        // SKIP ME FOR NOW
        //step 2: test if 


        if (g_all_is_well) {
            g_all_is_well = read_sample_pts(g_rbif_pts_filename, g_sample_pts, g_sample_pt_indices);
            if (g_all_is_well) {
                debug_print_sample_points();
            } else {
                ROS_INFO("was not able to read identified RBIP sample points");
                //g_fname_index--; //re-use previous filename index
            }
        }
        //go to a safe starting pose:
        if (g_all_is_well) {
            move_srv.request.rtn_to_camera_pose = false;
            move_srv.request.go_to_initial_pose = true;
            l515_moves_client.call(move_srv);
            ROS_WARN("moving to intermediate pose...");
            move_srv.request.x_wrt_RBIP = (X_MAX + X_MIN) / 2.0;
            move_srv.request.y_wrt_RBIP = (Y_MAX + Y_MIN) / 2.0;
            move_srv.request.z_wrt_RBIP = 1.0; //(Z_MAX+Z_MIN)/2.0;     
            if (!l515_moves_client.call(move_srv)) {
                ROS_ERROR("failed service call; quitting");
                return 1;
            }
            move_srv.request.go_to_initial_pose = false;
            
            //cout<<"enter 1:";
            //cin>>g_ans;            
        }

        if (g_all_is_well) { //try to visit all sample points with Intel visual servoing
            int nsamps = g_sample_pts.size();

            ROS_WARN("stepping through %d key points with visual servoing",nsamps);
            Point2d pt;
            int index_samp;

            robot_points_file.open(g_robot_pts_filename.c_str(), ios::out | ios::trunc);

            for (int isamp = 0; isamp < nsamps; isamp++) {
                pt = g_sample_pts[isamp];
                index_samp = g_sample_pt_indices[isamp];
                move_srv.request.x_wrt_RBIP = pt.x;
                move_srv.request.y_wrt_RBIP = pt.y;
                move_srv.request.z_wrt_RBIP = L515_INSPECTION_HEIGHT;
                ROS_INFO("sending robot to x,y,z = %f, %f, %f",move_srv.request.x_wrt_RBIP,
                       move_srv.request.y_wrt_RBIP, move_srv.request.z_wrt_RBIP);
                if (!l515_moves_client.call(move_srv)) {
                    ROS_ERROR("failed service call; quitting");
                    return 1;
                }
                ROS_INFO("l515 service returned");
                ros::Duration(2.0).sleep();
                //cout<<"enter 1:";
                //cin>>g_ans;
                ROS_WARN("start visual servoing ...");

                double convergence_err = 1.0; //skip the visual servoing...set err to 0.0 
                int ntries = 0;
                double dx, dy;
                while ((convergence_err > VISUAL_SERVO_CONVERGENCE_TOL)&&(ntries < VISUAL_SERVO_MAX_TRIES)) {
                    ntries++;
                    ROS_INFO("iteration %d", ntries);
                    ROS_INFO(" acquiring new image...");
                    g_got_new_image = false;
                    while (!g_got_new_image) {
                        ROS_INFO("waiting for image...");
                        ros::spinOnce();
                        ros::Duration(0.5).sleep();
                        ros::spinOnce();
                    }
                    //image is in g_src
                    ROS_INFO("searching for key point");

                    cv::Point2d tp = sc::detect_single_corner(g_src);

                    ROS_INFO("Found corner location (%f, %f)", tp.x, tp.y);
                    // * .see image corner_result.png\n\n"

                    cv::drawMarker(
                            g_src,
                            tp,
                            cv::Scalar(255, 0, 0), //Marker color
                            cv::MARKER_TILTED_CROSS, 25//Type and size.
                            );
                    string processed_fname = "corner_result.png";
                    imwrite(processed_fname, g_src);

                    compute_move_to_feature_center(tp, dx, dy);
                    convergence_err = sqrt(dx * dx + dy * dy);
                    ROS_INFO("computed visual servo correction: dx, dy = %f, %f", dx, dy);

                    if (convergence_err > VISUAL_SERVO_CONVERGENCE_TOL) {

                        ROS_INFO("moving the robot: ");
                        move_srv.request.x_wrt_RBIP += dx;
                        move_srv.request.y_wrt_RBIP += dy;
                        move_srv.request.z_wrt_RBIP = L515_INSPECTION_HEIGHT;
                        move_srv.request.z_rot_angle_increment[0] = L515_THETAZ;
                        move_srv.request.rtn_to_camera_pose = false;

                        l515_moves_client.call(move_srv);
                        ros::Duration(1.0).sleep();
                    } //done with a move

                } //end of visual servoing iterations
                ROS_INFO("ceased visual servoing iterations for this keypoint; saving pose info");
                if (convergence_err < VISUAL_SERVO_CONVERGENCE_TOL) {
                  robot_points_file << index_samp << ", " << move_srv.request.x_wrt_RBIP << ", " << move_srv.request.y_wrt_RBIP << endl;
                } //only save data if visual servoing was successful
            }
            ROS_INFO("done examining all key points for this poster");
            robot_points_file.close();
            //end of visual servoing to all sample points



            ROS_WARN("returning to camera pose");
            move_srv.request.rtn_to_camera_pose = true;
            l515_moves_client.call(move_srv);

            ROS_INFO("done with this poster");
            if (g_all_is_well) {
                g_fname_index++;
            }

        }
    }
    return 0;
}




