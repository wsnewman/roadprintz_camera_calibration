//v2: consult surface map to get metric coords w/rt RBIP for a pt near center of checkerboard


//node to test camera calibration using 5x7 small poster
//hosts a service for snapshot, move, rtn to camera pose
//has a client of motion-control service to communicate with node: test_moves/test_laser_moves_RBIP_frame

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <test_moves/TestMoveSrv.h>
#include <rbip_calibration/CamPosterCalibTestSrv.h>
#include <rbip_calibration/openCV_utils.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>

//MAGIC NUMBERS:

//virtual image values (after perspective transform)
//where to get these...must be consistent w/ perspective transform node;
//parameter server?  publish by perspective transform node? put in an include file associated w/ perspective transformer?
//launch file frame publisher for virtual_camera_frame?

#include <camera_parameters/virtual_cam_params.h>

#include <pcl_generator/MapperSrv.h> //to access mapper service

string g_image_topic = "/virtual_camera/image_rect_color";
/*
//location of virtual camera: 1.45 meters back and 3.0 meters high
const double VERT_CAM_HEIGHT=3.0; //this actually makes NO difference; but need some value here 
const double VIRT_CAM_X_OFFSET=1.45; //choose this to be near the physical camera offset
//scaling of virtual camera; can only do this w/ RBIP plane; equivalent to defining a focal length
const double KPIX = 590.0; //pixels per meter for virtual image; MUST MAKE THIS SAME AS IN PERSPECTIVE_TRANSFORM NOD
*/

const double PATCH_HALFWIDTH=0.1; //examine +/- this much in x and y direction for patch about nominal x,y

double cx_virt = g_virt_image_width/2.0;
double cy_virt = g_virt_image_height/2.0;



OpenCvUtils *g_open_cv_utils_ptr;

using namespace std;
int g_ans=0;
ros::ServiceClient *g_client_ptr;
ros::ServiceClient *g_mapper_client_ptr;

double g_target_x=1.5;
double g_target_y=0.0;
double g_target_z=0.0;
bool g_got_target=false;

string g_image_filename;

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
bool compute_target_from_pixels(double cam_x,double cam_y, double &target_x, double &target_y, double &target_z) {
    ROS_INFO(" compute_target_from_pixels: cam_x, cam_y = %f, %f",cam_x, cam_y);
    //given camera pixels, compute the corresponding point projected onto the RBIP plane, in the virtual-camera frame:
    Eigen::Vector3d pt_on_RBIP_plane_wrt_virt_cam;
    //here is a point projected onto the RBIP plane:
    pt_on_RBIP_plane_wrt_virt_cam[2] = VERT_CAM_HEIGHT;
    pt_on_RBIP_plane_wrt_virt_cam[0] = (cam_x-cx_virt)/KPIX;
    pt_on_RBIP_plane_wrt_virt_cam[1] = (cam_y-cy_virt)/KPIX;
    ROS_INFO_STREAM("pt_on_RBIP_plane_wrt_virt_cam: "<<pt_on_RBIP_plane_wrt_virt_cam.transpose()<<endl);
    Eigen::Vector3d pt_on_RBIP_plane_wrt_RBIP;
    pt_on_RBIP_plane_wrt_RBIP =   g_affine_virt_cam_wrt_RBIP*pt_on_RBIP_plane_wrt_virt_cam;
    ROS_INFO_STREAM("pt_on_RBIP_plane_wrt_RBIP: "<<pt_on_RBIP_plane_wrt_RBIP.transpose()<<endl);
    
    //now, find the height of this point:
    //use x,y on plane as near to center of small poster; use this to find a patch about this point
    //populate a service request to interpolate z(x,y)
        pcl_generator::MapperSrv map_srv;
        map_srv.request.request_code = pcl_generator::MapperSrvRequest::TEST_SPECIFIED_PATCH;
        ROS_INFO("sending request to mapper service, code %d",(int) map_srv.request.request_code);
        
        map_srv.request.x_max=  pt_on_RBIP_plane_wrt_RBIP[0] + PATCH_HALFWIDTH; //set tolerance for min patch size
        map_srv.request.x_min =pt_on_RBIP_plane_wrt_RBIP[0] - PATCH_HALFWIDTH;
        map_srv.request.y_max=  pt_on_RBIP_plane_wrt_RBIP[1] + PATCH_HALFWIDTH; //set tolerance for min patch size
        map_srv.request.y_min =pt_on_RBIP_plane_wrt_RBIP[1] - PATCH_HALFWIDTH;
        ROS_WARN(" calling map service w/ mode TEST_SPECIFIED_PATCH");
        if (!g_mapper_client_ptr->call(map_srv)) {
            ROS_WARN("surface map service request failed!");
            //response.success=false;
            return false;            
        }
        bool response_code= map_srv.response.response_code;
        if (response_code!=pcl_generator::MapperSrvResponse::SUCCESS) {
            ROS_WARN("response from surface map service for patch planar fit was FAILURE");
            //response.success=false;
            return false;
        }
        //if good to here, charge on;
        Eigen::Vector3d plane_normal;
        double plane_offset = map_srv.response.plane_dist;
        ROS_INFO("plane offset = %f",plane_offset);
        plane_normal[0]=map_srv.response.planar_normal_x;
        plane_normal[1]=map_srv.response.planar_normal_y;
        plane_normal[2]=map_srv.response.planar_normal_z;
        
        ROS_INFO_STREAM("patch plane offset = "<< plane_offset<<" normal = "<<plane_normal.transpose()<<endl); 
        
        
    //now, find intersection of optical ray with identified plane:
    // given O_cam/RBIP:
        Eigen::Vector3d O_cam_wrt_RBIP = g_affine_virt_cam_wrt_RBIP.translation();
    // pts along optical vector the optical vector from focal point, through (u,v), must lie along:
    // p/RBIP = O_cam/RBIP + l*[dy_cam/(H*Kpix); dx_cam/(H*Kpix); -1], where "l" is the dist from focal pt along optical ray
    //in the above, transform to RBIP is implicit, involving swapping u and v
    // and this must intersect a plane at specified normal and offset (also w/rt RBIP)
    // any point p/RBIP that lies on the plane must satisfy: d = nhat dot p
    // therefore, d = nhat (dot) {O_cam/RBIP + l*[v/(H*Kpix); u/(H*Kpix); -1] }
    // solve for l: l = (d- nhat (dot) {O_cam/RBIP)/nhat (dot) O_cam/RBIP)/(nx*v/(H*Kpix) + ny*u/(H*Kpix) - nz)
    // use l to compute p/RBIP = O_cam/RBIP + l*[v/(H*Kpix); u/(H*Kpix); -1]
    Eigen::Vector3d opt_vec;
    
    opt_vec[0] =     (cam_y-cy_virt)/(KPIX*VERT_CAM_HEIGHT);
    opt_vec[1] =     (cam_x-cx_virt)/(KPIX*VERT_CAM_HEIGHT);
    opt_vec[2] = -1.0;
    ROS_INFO_STREAM("optical vector: "<<opt_vec.transpose()<<endl);
    double veclen = (plane_offset - plane_normal.dot(O_cam_wrt_RBIP))/(plane_normal.dot(opt_vec));
    ROS_INFO("veclen = %f",veclen);
    Eigen::Vector3d pt_on_poster_wrt_RBIP;
    pt_on_poster_wrt_RBIP = O_cam_wrt_RBIP+veclen*opt_vec;
    ROS_INFO_STREAM("pt_on_poster_wrt_RBIP: "<<pt_on_poster_wrt_RBIP.transpose()<<endl);
        
        
        
    target_x= pt_on_poster_wrt_RBIP[0]; //1.5;
    target_y=pt_on_poster_wrt_RBIP[1];//0.0;
    target_z = pt_on_poster_wrt_RBIP[2];
    return true;
}


bool calibCb(rbip_calibration::CamPosterCalibTestSrvRequest& request, rbip_calibration::CamPosterCalibTestSrvResponse& response) {
    int cmd_code =  request.cmd_code;
    
 double des_x, des_y;
 geometry_msgs::Point point_msg;    
    ROS_INFO("requested command code = %d",cmd_code);
    test_moves::TestMoveSrv srv;
    if (cmd_code==rbip_calibration::CamPosterCalibTestSrvRequest::GO_TO_CAMERA_POSE) {
          srv.request.rtn_to_camera_pose=true;
          g_client_ptr->call(srv);
          if (srv.response.success==true) {
            ROS_INFO("returning to camera pose");
            response.err_code = rbip_calibration::CamPosterCalibTestSrvResponse::OK;
            response.success=true;
            return true;
          }
          else {
              ROS_WARN("test moves service responded to camera_pose request with failure");
            response.success=false;
            return true;
          }
        }
    
    else if (cmd_code==rbip_calibration::CamPosterCalibTestSrvRequest::TAKE_SNAPSHOT) {
        ROS_INFO("attempting to take snapshot and find poster in image...not ready");
        g_x_pix_upper_left=0;
        g_y_pix_upper_left=0;
        g_x_pix_keypt=0;
        g_y_pix_keypt=0;
        
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
        /*
        if (!g_preferred_orientation) {
            ROS_WARN("poster found, but not in preferred orientation; will not trust");
           response.success=false;
           return true;              
        }*/
        //g_x_upper_left, g_y_upper_left
        ROS_INFO("found poster; inner key pt: %f, %f",g_x_pix_keypt,g_y_pix_keypt);
        imwrite(g_image_filename,g_copy_of_image_markup); //ugh: result in global var



        g_got_target = compute_target_from_pixels(g_x_pix_keypt,g_y_pix_keypt, g_target_x, g_target_y,g_target_z);

        if (!g_got_target) { ROS_WARN("problem converting pixels to meters"); 
           response.success=false;
           return true; 
        }
        //else {
            ROS_INFO("pixels converted to meters: target_x, target_y, target_z = %f, %f, %f",g_target_x,g_target_y,g_target_z);
        //}

        response.success=true;
        return true;       
    }
    else if (cmd_code==rbip_calibration::CamPosterCalibTestSrvRequest::GO_TO_TARGET_POSE) {
        ROS_INFO("attempting to point at target...careful!");
        //g_got_target = compute_target_from_pixels(g_cam_x,g_cam_y, g_target_x, g_target_y);
        if (g_got_target) {
            ROS_INFO("sending robot to x,y = %f, %f",g_target_x, g_target_y);
          srv.request.rtn_to_camera_pose=false;
          srv.request.x_wrt_RBIP= g_target_x;
          srv.request.y_wrt_RBIP= g_target_y;
          srv.request.z_wrt_RBIP= g_target_z;
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
    
    ros::ServiceClient map_client = nh.serviceClient<pcl_generator::MapperSrv>("surface_map_service");
    g_mapper_client_ptr = &map_client;

    ros::ServiceServer service = nh.advertiseService("cam_poster_calib_test_service", calibCb);
    OpenCvUtils open_cv_utils(g_image_topic);  //this starts up an image process that constantly looks for 
                               //poster fits, defaulting to small poster
    g_open_cv_utils_ptr= &open_cv_utils;
    ImageTransportUtil ic;
    
    string ros_ws_path = getenv("ROS_WORKSPACE");
    g_image_filename = ros_ws_path + "/stella_calib_data/small_poster_markup.png";
    
    ros::spin();

  return 0;
}

  


