//test main--to be replaced by GUI


#include <ros/ros.h>
#include <rbip_calibration/CamPosterCalibTestSrv.h>
#include <test_moves/TestMoveSrv.h>
#include <rbip_calibration/openCV_utils.h>

#include <geometry_msgs/Point.h>

using namespace std;
int g_ans=0;


ros::ServiceClient *g_client_ptr;

OpenCvUtils g_open_cv_utils;


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
        
        
    ROS_WARN("rest of callback not implemented");
        
        response.success=false;
        return true;
    }

    



int main(int argc, char** argv)
{
  ros::init(argc, argv, "interactive_image_gui_test_main");
  ros::NodeHandle nh;

    rbip_calibration::CamPosterCalibTestSrv srv;
    
    ros::ServiceClient client = nh.serviceClient<rbip_calibration::CamPosterCalibTestSrv>("cam_poster_calib_test_service");

 //std_msgs::Bool rtn_bool;
 //rtn_bool.data=1;
 int cmd_code;
  while(ros::ok()) {
    cout<<"enter 0 to go to camera pose, 1 to take a snapshot, 2 to go to target pose: ";
    cin>>cmd_code;
    if ((cmd_code<0)||(cmd_code>2)) {
        ROS_WARN("not a valid code");
    }

    else if (cmd_code==0) {
          srv.request.cmd_code=   rbip_calibration::CamPosterCalibTestSrvRequest::GO_TO_CAMERA_POSE;
          client.call(srv);
        }
    else if (cmd_code==1) {
          srv.request.cmd_code=   rbip_calibration::CamPosterCalibTestSrvRequest::TAKE_SNAPSHOT;
          client.call(srv);            
        }
    else if (cmd_code==2) {
          srv.request.cmd_code=   rbip_calibration::CamPosterCalibTestSrvRequest::GO_TO_TARGET_POSE;
          client.call(srv);            
        }       
    }
    /*
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
  }*/
    
  return 0;
}

  


