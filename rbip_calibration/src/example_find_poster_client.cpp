//example client node for interacting with service in find_poster_in_lidar_image_10x7_service


#include <ros/ros.h>
#include <rbip_calibration/CamPosterCalibTestSrv.h>
#include <iostream>
#include <string>


using namespace std;
int g_ans;


int main(int argc, char **argv) {
    ros::init(argc, argv, "find_poster_client");
    ros::NodeHandle n;
    //rbip_calibration::CamPosterCalibTestSrvRequest& request
    ros::ServiceClient client = n.serviceClient<rbip_calibration::CamPosterCalibTestSrv>("find_poster_service");
    rbip_calibration::CamPosterCalibTestSrv srv;
    //srv.fname = "test_fname";

        ROS_INFO("Requesting find poster");
        string modifier;
        cout<<"enter example filename to use: ";
        cin>>modifier;
        cout<<"got name = "<<modifier<<endl;        
        srv.request.fname = modifier.c_str();
        
        client.call(srv);
 

    return 0;
}
