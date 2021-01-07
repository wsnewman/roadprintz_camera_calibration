//test main--to be replaced by GUI


#include <ros/ros.h>
#include <rbip_calibration/CamPosterCalibTestSrv.h>

using namespace std;
    
int main(int argc, char** argv)
{
  ros::init(argc, argv, "interactive_image_gui_test_main");
  ros::NodeHandle nh;

    rbip_calibration::CamPosterCalibTestSrv srv;
    
    ros::ServiceClient client = nh.serviceClient<rbip_calibration::CamPosterCalibTestSrv>("cam_poster_calib_test_service");

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
    
  return 0;
}

  


