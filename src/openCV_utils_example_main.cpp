//example main for using openCV utils:

#include <roadprintz_camera_calibration/openCV_utils.h>

OpenCvUtils g_open_cv_utils; //instantiate object of class OpenCvUtils, global object

int main(int argc, char **argv) {
    ros::init(argc, argv, "open_cv_utils_test_main");
    ros::NodeHandle nh;

    cout<<"enter image filename, WITHOUT .png: ";
    string fname_root;
    cin>>fname_root;

    //in this test main, set the path to the current directory:
    string path = "."; //current directory

    int n_keypts_wide=8;
    int n_keypts_high=6;
    //        bool find_poster(string path, string fname_image_root,int n_keypts_wide, int n_keypts_high, bool &preferred_orientation, double &x_upper_left, double &y_upper_left);

    bool preferred_orientation;
    double x_upper_left,y_upper_left;
    if (!g_open_cv_utils.find_poster(path,fname_root, n_keypts_wide, n_keypts_high,preferred_orientation,x_upper_left, y_upper_left)) {
      ROS_WARN("find_poster() was NOT successful!");
      return 1;
    }
     ROS_INFO("call to find_poster() was successful");
    if (preferred_orientation) ROS_INFO("and checkerboard is in preferred orientation");
    else ROS_WARN("identified checkerboard keypoints are not in preferred order");
    ROS_INFO("x,y cam coords of upper-left key point: %f, %f",x_upper_left,y_upper_left);
  return 0;
}


