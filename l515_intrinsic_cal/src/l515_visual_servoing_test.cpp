//this node does not belong in this package
// it is intended for a quick test of visual servoing, to be extended to automated camera calibration
// It should:
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

using namespace std;
using namespace cv;
int g_ans = 0;

string g_input_image_topic="/camera/color/image_raw"; //set input topic name
Mat g_src;
bool g_got_new_image=false;

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

string g_dir_name;
vector<string> g_image_names;

vector<double> g_scan_vec_of_dbls;
bool g_got_lidar_data=false;
//careful near robot base; interferes with self


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
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

    g_src = cv_ptr->image;
    g_got_new_image=true; //note that new image has been acquired
}


string gen_fname(int i, int j, int k, int r) {
    stringstream ss;
    ss << g_dir_name;
    ss<<"image_"<<k<<"_"<<j<<"_"<<i<<"_"<<r<<".png";
    string str = ss.str();
    return str;
}

void set_sample_poses(vector<Eigen::VectorXd> &sample_poses,vector<string> &fnames) {
    sample_poses.clear();
    Eigen::VectorXd pose;
    pose.resize(4);
    //double x,y,z;
    int i,j,k;
    int i_max = round((X_MAX-X_MIN)/DX);
    int j_max = round((Y_MAX-Y_MIN)/DY);
    int k_max = round ((Z_MAX-Z_MIN)/DZ);
    
    //outfile.open(pose_info);
    //pose[2] = SCAN_HEIGHT;
    //fill and stuff sample_poses with vectors to visit
    for (k=0;k<=k_max;k++) {
        for (j=0;j<=j_max;j++) {
            for (i=0;i<=i_max;i++) {
                for (int r=0;r<4;r++) {
                    pose[0] = X_MIN+i*DX;
                    pose[1]= Y_MIN+j*DY;
                    pose[2]= Z_MIN+k*DZ;
                    pose[3] = r*M_PI/2.0;
                    sample_poses.push_back(pose);
                    fnames.push_back(gen_fname(i,j,k,r));
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
    
    g_got_lidar_data=false;
    while (!g_got_lidar_data) {
        ros::spinOnce();
     ROS_INFO("waiting for scan data");
     ros::Duration(0.5).sleep();
    }
    
    
}

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
    dx = (v_fid-L515_ROT_CTR_VC)/L515_PIX_PER_METER;
    dy = (u_fid-L515_ROT_CTR_UC)/L515_PIX_PER_METER;
    return true; //should have some error handling
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
        

    ros::ServiceClient client = nh.serviceClient<test_moves::TestMoveSrv>("l515_moves_service");
    
    //string ros_ws_path = getenv("ROS_WORKSPACE"); 
    //string result_file = ros_ws_path + "/stella_calib_data/stella_lidar_data/l515_scan_data_file.csv";
    //g_dir_name = ros_ws_path + "/stella_calib_data/l515_extrinsic_data/";
    //string pose_info_file = g_dir_name+"pose_info.txt";
    //ofstream outfile;
    //outfile.open(pose_info_file); 
    //outfile<< "image file names encode _k_j_i, where x = x_min+i*dx, y = y_min+j*dy, z = z_min+k*dz"<<endl;
    //outfile << "X_MIN = "<<X_MIN<<", X_MAX = "<<X_MAX<<", DX = "<<DX<<endl;
    //outfile << "Y_MIN = "<<Y_MIN<<", Y_MAX = "<<Y_MAX<<", DY = "<<DY<<endl;
    //outfile << "Z_MIN = "<<Z_MIN<<", Z_MAX = "<<Z_MAX<<", DZ = "<<DZ<<endl;
    //outfile.close();
    
    
    test_moves::TestMoveSrv srv;
    srv.request.z_rot_angle_increment.resize(1);
    srv.request.z_rot_angle_increment[0]=L515_THETAZ;
    srv.request.z_wrt_RBIP = L515_INSPECTION_HEIGHT;        
            
    srv.request.rtn_to_camera_pose = false; //first call will NOT be return to camera pose


    //vector<Eigen::VectorXd> sample_poses;
    //Eigen::Vector3d measured_robot_pose;
    //vector<string> image_fnames;
    //string image_fname;
    //void set_sample_poses(vector<Eigen::Vector3d> &sample_poses,vector<string> image_fnames) {

    //set_sample_poses(sample_poses,image_fnames);
    
    //step through the sample poses and record measurements
    //int nposes = sample_poses.size();
    //ROS_INFO("will cycle through %d poses", nposes);
    //ROS_INFO("file names: ");
    //Eigen::VectorXd pose;
    //pose.resize(4);
    //for (int i=0;i<nposes;i++) {
    //    pose = sample_poses[i];
    //    cout<<image_fnames[i]<<" pose: x,y,z,thetaz = "<<pose[0]<<", "<<pose[1]<<", "<<pose[2]<< ", "<<pose[3]<<endl;
    //}
    
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(g_input_image_topic.c_str(), 1, imageCallback);
    //test the camera:
    ROS_INFO("testing the camera...");
    g_got_new_image=false;
        while (!g_got_new_image) {
            ROS_INFO("waiting for image...");
            ros::spinOnce();
            ros::Duration(0.5).sleep();
            ros::spinOnce();
        }
    ROS_INFO("got an image; should be able to view on camera topic");
            
    cout << "enable the robot and enter 1 to move to initial reference pose: ";
    cin>>g_ans;
        srv.request.x_wrt_RBIP = (X_MAX+X_MIN)/2.0;
        srv.request.y_wrt_RBIP = (Y_MAX+Y_MIN)/2.0;
        srv.request.z_wrt_RBIP = L515_INSPECTION_HEIGHT; //(Z_MAX+Z_MIN)/2.0;     
        if (!client.call(srv)){
            ROS_ERROR("failed service call; quitting");
            return 1;
        }    
        
        ROS_INFO("E-stop the robot, place the poster with a keypoint in the camera view");
        ROS_INFO(" then enter 1 to resume: ");
    cin>>g_ans;
    double dx, dy;
    
    
    ROS_INFO("starting sequence for visual servoing");
    while (ros::ok()) {
    //get an image:
        ROS_INFO(" acquiring new image...");
        g_got_new_image=false;
        while (!g_got_new_image) {
            ROS_INFO("waiting for image...");
            ros::spinOnce();
            ros::Duration(0.5).sleep();
            ros::spinOnce();
        }    
        //image is in g_src
        ROS_INFO("searching for key point");
        
	//cv::Mat img = cv::imread(argv[1]);
	
	cv::Point2d tp = sc::detect_single_corner(g_src);
	
	printf("\nFound corner location (%f, %f).see image corner_result.png\n\n", tp.x, tp.y);
	
	cv::drawMarker(
		g_src,
		tp,
		cv::Scalar(255, 0, 0),//Marker color
		cv::MARKER_TILTED_CROSS, 25//Type and size.
	);
	string processed_fname = "corner_result.png";
        imwrite(processed_fname,g_src);
        
        compute_move_to_feature_center(tp, dx, dy);
        ROS_INFO("computed visual servo correction: dx, dy = %f, %f",dx,dy);
        cout<<"enter 0 to skip this move (move poster, if desired, before entering 0)"<<endl;
        cout<<"enter 1 to make this move"<<endl;
        cout<<"enter 2 to move to camera pose and quit : ";
        cout<<"enter 3 to rotate about this point : ";
        cin>>g_ans;
        if (g_ans==2) {
                srv.request.rtn_to_camera_pose = true;
                client.call(srv);
    
                ROS_INFO("test node concluded");
                return 0;
        }
        if (g_ans==1) {
            ROS_INFO("moving the robot: ");
            srv.request.x_wrt_RBIP += dx;
            srv.request.y_wrt_RBIP += dy;
            //srv.request.z_wrt_RBIP = L515_INSPECTION_HEIGHT;
            //srv.request.z_rot_angle_increment[0]=L515_THETAZ;
            srv.request.rtn_to_camera_pose = false;
            
            client.call(srv);
            ros::Duration(1.0).sleep();
        }     
        if (g_ans==3) {
            ROS_INFO("testing rotations");
            for (int irot=0;irot<4;irot++) {
                ROS_INFO("rot %d",irot);
                
            
                srv.request.z_rot_angle_increment.resize(1);  
                srv.request.rtn_to_camera_pose = false;
                srv.request.z_rot_angle_increment[0]= irot*M_PI/2.0;
                if (!client.call(srv)) {
                    ROS_ERROR("failed service call; quitting");
                return 1;
                }
                ros::Duration(2.0).sleep(); 
                        ROS_INFO(" acquiring new image...");
        g_got_new_image=false;
        while (!g_got_new_image) {
            ROS_INFO("waiting for image...");
            ros::spinOnce();
            ros::Duration(0.5).sleep();
            ros::spinOnce();
        }    
        ROS_INFO("got new image");
        //image is in g_src
        ROS_INFO("searching for key point");
                tp = sc::detect_single_corner(g_src);
	
	        ROS_INFO("rot = %f; Found corner location (%f, %f).see image corner_result.png\n\n", srv.request.z_rot_angle_increment[0], tp.x, tp.y);
                        compute_move_to_feature_center(tp, dx, dy);
            ROS_INFO("computed visual servo correction: dx, dy = %f, %f",dx,dy);
            
            }                
               
            }
            
        }

    return 0;
}




