//node to test camera calibration using 5x7 small poster
//hosts a service for snapshot, move, rtn to camera pose
//has a client of motion-control service to communicate with node: test_moves/test_laser_moves_RBIP_frame

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <test_moves/TestMoveSrv.h>
#include <roadprintz_camera_calibration/CamPosterCalibTestSrv.h>
#include <roadprintz_camera_calibration/openCV_utils.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>

//MAGIC NUMBERS:
    //intrinsics (of original camera);
//THESE SHOULD GO AWAY
//double g_fx = 1637.367343; 
//double g_fy = 1638.139550;   
//double g_cx = 1313.144667;
//double g_cy = 774.029343;
//don't use the above; use instead values for virtual (ideal) camera:

//virtual image values (after perspective transform)
//where to get these...must be consistent w/ perspective transform node;
//parameter server?  publish by perspective transform node? put in an include file associated w/ perspective transformer?
//launch file frame publisher for virtual_camera_frame?

const int g_image_width = 2688;
const int g_image_height = 1520; 

//ISSUE: in image, I am seeing more like 592 to 593 pixels/meter
double cx_virt = g_image_width/2.0;
double cy_virt = g_image_height/2.0;

//location of virtual camera: 1.45 meters back and 3.0 meters high
const double VERT_CAM_HEIGHT=3.0; //this actually makes NO difference; but need some value here 
const double VIRT_CAM_X_OFFSET=1.45; //choose this to be near the physical camera offset
//scaling of virtual camera; can only do this w/ RBIP plane; equivalent to defining a focal length
const double KPIX = 590.0; //pixels per meter for virtual image; MUST MAKE THIS SAME AS IN PERSPECTIVE_TRANSFORM NOD

OpenCvUtils *g_open_cv_utils_ptr;

using namespace std;
int g_ans=0;
ros::ServiceClient *g_client_ptr;

//double g_cam_x=0;
//double g_cam_y=0;
double g_target_x=1.5;
double g_target_y=0.0;
bool g_got_target=false;



 Eigen::Affine3d g_affine_virt_cam_wrt_RBIP;



Eigen::Affine3d get_hardcoded_affine_virtual_cam_wrt_RBIP(void) {
	Eigen::Affine3d hardcoded_affine_cam_wrt_RBIP;

	Eigen::Vector3d trans;
	Eigen::Matrix3d R;

        trans<<  VIRT_CAM_X_OFFSET, 0.0,    VERT_CAM_HEIGHT; //choose virt cam offset close to physical cam offset
    //note: camera origin is about 2.885 meters high, and about 2.768m aft of rear wheels
    //   camera origin is nearly along vehicle centerline, but shifted about 7mm to starboard

     //ideal orientation: normal to ideal plane
     R <<     0,    1,   0,
              1,    0,   0,
              0,    0,  -1;
    //note: by construction,
    // virtual camera x-axis is parallel to RBIP y-axis
    //         camera y-axis is parallel to RBIP x-axis
    //         camera z-axis is antiparallel to RBIP z-axis
    

	hardcoded_affine_cam_wrt_RBIP.linear() = R;
	hardcoded_affine_cam_wrt_RBIP.translation() = trans;

	return hardcoded_affine_cam_wrt_RBIP;
}

//here's the main deal
//assume images are virtual images that have been rectified and perspective transformed
//also assume knowledge of the virtual camera properties: 
bool compute_target_from_pixels(double cam_x,double cam_y, double &target_x, double &target_y) {
    ROS_INFO(" compute_target_from_pixels: cam_x, cam_y = %f, %f",cam_x, cam_y);
    //given camera pixels, compute the corresponding point projected onto the RBIP plane, in the virtual-camera frame:
    Eigen::Vector3d pt_wrt_virt_cam;
    pt_wrt_virt_cam[2] = VERT_CAM_HEIGHT;
    pt_wrt_virt_cam[0] = (cam_x-cx_virt)/KPIX;
    pt_wrt_virt_cam[1] = (cam_y-cy_virt)/KPIX;
    ROS_INFO_STREAM("pt_wrt_virt_cam: "<<pt_wrt_virt_cam.transpose()<<endl);
    Eigen::Vector3d pt_wrt_RBIP;
    pt_wrt_RBIP =   g_affine_virt_cam_wrt_RBIP*pt_wrt_virt_cam;
    ROS_INFO_STREAM("pt_wrt_RBIP: "<<pt_wrt_RBIP.transpose()<<endl);
    
    target_x=pt_wrt_RBIP[0]; //1.5;
    target_y=pt_wrt_RBIP[1];//0.0;
    return true;
}


bool calibCb(roadprintz_camera_calibration::CamPosterCalibTestSrvRequest& request, roadprintz_camera_calibration::CamPosterCalibTestSrvResponse& response) {
    int cmd_code =  request.cmd_code;
    
 double des_x, des_y;
 geometry_msgs::Point point_msg;    
    ROS_INFO("requested command code = %d",cmd_code);
    test_moves::TestMoveSrv srv;
    if (cmd_code==roadprintz_camera_calibration::CamPosterCalibTestSrvRequest::GO_TO_CAMERA_POSE) {
          srv.request.rtn_to_camera_pose=true;
          g_client_ptr->call(srv);
          if (srv.response.success==true) {
            ROS_INFO("returning to camera pose");
            response.err_code = roadprintz_camera_calibration::CamPosterCalibTestSrvResponse::OK;
            response.success=true;
            return true;
          }
          else {
              ROS_WARN("test moves service responded to camera_pose request with failure");
            response.success=false;
            return true;
          }
        }
    
    else if (cmd_code==roadprintz_camera_calibration::CamPosterCalibTestSrvRequest::TAKE_SNAPSHOT) {
        ROS_INFO("attempting to take snapshot and find poster in image...not ready");
        g_x_pix_upper_left=0;
        g_y_pix_upper_left=0;
        g_found_poster=false; //init assumption no poster found
        g_preferred_orientation=false; //init assumption, unless proven otherwise        
        g_got_new_image=false; //trigger to request a new image
        while (!g_got_new_image) {
            ros::Duration(0.5).sleep();
            ros::spinOnce();
            ROS_INFO("waiting for new image capture...");
        }
        if (!g_found_poster) {
            ROS_WARN("poster not found in image");
           response.success=false;
           return true;             
        }
        if (!g_preferred_orientation) {
            ROS_WARN("poster found, but not in preferred orientation; will not trust");
           response.success=false;
           return true;              
        }
        //g_x_upper_left, g_y_upper_left
        ROS_INFO("found poster; corner pixel values are: %f, %f",g_x_pix_upper_left,g_y_pix_upper_left);
        g_got_target = compute_target_from_pixels(g_x_pix_upper_left,g_y_pix_upper_left, g_target_x, g_target_y);
        if (!g_got_target) { ROS_WARN("problem converting pixels to meters"); }
        else {
            ROS_INFO("pixels converted to meters: target_x, target_y = %f, %f",g_target_x,g_target_y);
        }
        //save image:
        string processed_image_fname = "poster_markup.png"; //path+"/"+fname_image_root+"_processed.png";
        imwrite(processed_image_fname,g_copy_of_image_markup); //ugh: result in global var
         response.success=true;
        return true;       
    }
    else if (cmd_code==roadprintz_camera_calibration::CamPosterCalibTestSrvRequest::GO_TO_TARGET_POSE) {
        ROS_INFO("attempting to point at target...careful!");
        //g_got_target = compute_target_from_pixels(g_cam_x,g_cam_y, g_target_x, g_target_y);
        if (g_got_target) {
            ROS_INFO("sending robot to x,y = %f, %f",g_target_x, g_target_y);
          srv.request.rtn_to_camera_pose=false;
          srv.request.x_wrt_RBIP= g_target_x;
          srv.request.y_wrt_RBIP= g_target_y;
          g_client_ptr->call(srv);
            response.success=srv.response.success;
            return true; 
        }
         response.success=false;
        return true;       
    }        
        
    ROS_WARN("rest of callback not implemented");
        
        response.success=false;
        return true;
    }

    



int main(int argc, char** argv)
{
  ros::init(argc, argv, "interactive_image_gui");
  ros::NodeHandle nh;

  g_affine_virt_cam_wrt_RBIP = get_hardcoded_affine_virtual_cam_wrt_RBIP();

    ros::ServiceClient client = nh.serviceClient<test_moves::TestMoveSrv>("test_moves_service");
    g_client_ptr = &client;
    //test_moves::TestMoveSrv srv;  
    //srv.request.rtn_to_camera_pose=false; //first call will NOT be return to camera pose

    ros::ServiceServer service = nh.advertiseService("cam_poster_calib_test_service", calibCb);
    OpenCvUtils open_cv_utils;
    g_open_cv_utils_ptr= &open_cv_utils;
    ImageTransportUtil ic;
    
    ros::spin();

 //std_msgs::Bool rtn_bool;
 //rtn_bool.data=1;
 /*
  while(ros::ok()) {
    cout<<"enter desired x value: ";
    cin>>des_x;
    cout<<"enter desired y value: ";
    cin>>des_y;
    cout<<"enter 1 to send motion command: ";
    cin>>g_ans;
    if (g_ans==1) {
       ROS_INFO("sending motion command service request");
          srv.request.rtn_to_camera_pose=false;
          srv.request.x_wrt_RBIP= des_x;
          srv.request.y_wrt_RBIP= des_y;
 	  client.call(srv);
          ros::Duration(2.0).sleep();

          cout<<"enter 1 to return to camera pose: ";
          cin>>g_ans;
          srv.request.rtn_to_camera_pose=true;
          client.call(srv);
    }
  }
    */
  return 0;
}

  


