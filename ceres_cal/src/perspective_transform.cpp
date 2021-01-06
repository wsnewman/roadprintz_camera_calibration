//program to convert an input image into a top-down image, normal to ideal plane and aligned w/ x-y axes
// rqrs knowledge of input image extrinsics in terms of an affine transform

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <iostream>
#include <fstream>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace cv;
using namespace std;

const double RESCALE_FACTOR=2.0; //scale original image down by this factor to fit display size


//convert camera image into an ideal image, as viewed normal to the ideal plane
//SHOULD get the following values from camera_info...fix later


//numbers for Stella's camera:
const int g_image_width = 2688;
const int g_image_height = 1520;  
const double   g_image_xc = 1313.144667; //do these change  for rectified image?
const double   g_image_yc = 774.029343;
const double g_image_fx = 1637.367343;
const double g_image_fy = 1638.139550;

const double g_virt_image_width = 2688; //might want to change these if change resolution of virtual image
const double g_virt_image_height = 1520; 


//experiment with values to get desired pixel scaling, pix/meter
const double VERT_CAM_HEIGHT=3.0; //this actually makes NO difference; but need some value here 


const double KPIX = 590.0; //pixels per meter for virtual image; might want to change, e.g., to 1000.0

static const std::string OPENCV_WINDOW = "ImageWindow";

Eigen::Affine3d g_affine_virt_cam_wrt_cam;


cv::Mat g_virtual_image; //(Nu,Nv,CV_8U,cv::Scalar(0));
cv::Mat g_src;  // need access to this; subsribe?
sensor_msgs::ImagePtr g_image_msg_ptr;

//precompute how u_virt,v_virt map ont u_cam,v_cam
//hack: assign black to corner (0,0) in physical image, and
// for all (u_virt,v_virt) coordinates that do not have a correspondence in physical image,
// assign u_cam,v_cam = 0,0
int u_cam_mappings[g_image_width][g_image_height];
int v_cam_mappings[g_image_width][g_image_height];


bool g_got_new_image = false;
bool g_first_time = true;
int g_ans;

//issue here: topics to subscribe and publish
//HMI expects "/camera/image_rect_color" (actually, compressed version)
//but want to pipe this through this perspective transform first
std::string g_output_image_topic="/camera/image_rect_color";
std::string g_input_image_topic="/camera_pretransformed/image_rect_color";

//SHOULD get this from tf publication; hardcode for the present
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
     //   trans<<  2.76911, 0.00777921,    2.88495; //4.44
        trans<<2.7955,  0.0109847,    2.8659 ; //4.328155, new 12/3/20 after LIDAR calibration and improved base w/rt sys transform
    //note: camera origin is about 2.885 meters high, and about 2.768m aft of rear wheels
    //   camera origin is nearly along vehicle centerline, but shifted about 7mm to starboard

    // R <<     0.0525021,  0.995724,   0.0759998,
    //          0.9976,    -0.055738,   0.0410976,
    //          0.0451579,  0.0736596, -0.996261;
     //12/3/20:
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


//define an ideal transform for the virtual camera;
//may choose origin to be close to physical camera origin, so field of view is comparable
Eigen::Affine3d get_hardcoded_affine_virtual_cam_wrt_sys(void) {
	Eigen::Affine3d hardcoded_affine_cam_wrt_sys;

	Eigen::Vector3d trans;
	Eigen::Matrix3d R;
	Eigen::Vector3d xvec_cam_wrt_sys,yvec_cam_wrt_sys,zvec_cam_wrt_sys;
    //v<< 2.81, 0,  2.87; //vector from origin of system_ref_frame to camera frame
   
    //R <<  -1, 0,  -0,   //orientation of camera frame w/rt system_ref_frame, optical axis points in neg z direction
    //           0, 1, 0,
    //           0, 0, -1;   

     //   trans<<  2.76796, 0.00720234,    2.88546; //4.5426 rms pixel error in reprojection based on ~1,000 points
        trans<<  2.77, 0.0,    VERT_CAM_HEIGHT; //
    //note: camera origin is about 2.885 meters high, and about 2.768m aft of rear wheels
    //   camera origin is nearly along vehicle centerline, but shifted about 7mm to starboard

     //ideal orientation: normal to ideal plane
     R <<     0,    1,   0,
              1,    0,   0,
              0,    0,  -1;
    //note: camera x-axis is ~parallel to system y-axis
    //      camera y-axis is ~parallel to system x-axis
    //      camera z-axis is ~antiparallel to system z-axis
    

	hardcoded_affine_cam_wrt_sys.linear() = R;
	hardcoded_affine_cam_wrt_sys.translation() = trans;

	return hardcoded_affine_cam_wrt_sys;
}

bool valid_uv(int u,int v,int Nu,int Nv) {
  if (u<0) return false;
  if (v<0) return false;
  if (u>Nu-1) return false;
  if (v>Nv-1) return false;
  return true;
}

//pre-compute the mappings between (u,v) of virtual camera and (u,v) of physical camera
void compute_mappings(Eigen::Affine3d affine_virt_cam_wrt_cam, 
  double cx_src,double cy_src, double fx_src, double fy_src, double vert_height, double kpix)
{
    //int u_cam_mappings[g_image_width][g_image_height];
    //int v_cam_mappings[g_image_width][g_image_height];
    double cx_virt = g_image_width/2.0;
    double cy_virt = g_image_height/2.0;
    Eigen::Vector3d p_wrt_virt,p_wrt_cam;
  int u_cam,v_cam;
  p_wrt_virt[2] = vert_height;
 for (int u_virt=0;u_virt<g_image_width;u_virt++) {
  for (int v_virt=0;v_virt<g_image_height;v_virt++) {
      u_cam_mappings[u_virt][v_virt] = 0; //default
      v_cam_mappings[u_virt][v_virt] = 0;

    //convert u,v to x,y in virtual frame, as projected onto ideal plane
    p_wrt_virt[0]  = (u_virt-cx_virt)/kpix;
    p_wrt_virt[1]  = (v_virt-cy_virt)/kpix;
    //cout<<endl;
    //cout<<"u_virt, v_virt = "<<u_virt<<", "<<v_virt<<endl;
    //cout<<"p_wrt_virt = "<<p_wrt_virt[0]<<", "<<p_wrt_virt[1]<<", "<<p_wrt_virt[2]<<endl;
    p_wrt_cam = affine_virt_cam_wrt_cam*p_wrt_virt;
    //cout<<"p_wrt_cam = "<<p_wrt_cam[0]<<", "<<p_wrt_cam[1]<<", "<<p_wrt_cam[2]<<endl;
    u_cam = (int) (cx_src + fx_src*p_wrt_cam[0]/p_wrt_cam[2]);
    v_cam = (int) (cy_src + fy_src*p_wrt_cam[1]/p_wrt_cam[2]);
    //cout<<"u_cam, v_cam = "<<u_cam<<", "<<v_cam<<endl;
    if (valid_uv(u_cam,v_cam,g_image_width,g_image_height)) { 
        u_cam_mappings[u_virt][v_virt] = u_cam;
        v_cam_mappings[u_virt][v_virt] = v_cam;
    } 
  }
 }
    
}

  
//set camera image as src; set transformed image to be created as dst
//specify central pixel of input image, (cx_src,cy_src), and foci of input impage, (fx_src,fy_src)
//specify transform of virtual cam frame w/rt actual cam frame
//specify height of ideal, virtual image to be populated
//specify scaling of new iage: kpix = pixels/meter at ideal plane, from height vert_height
//create new, virtual (dst) image transformed from src image
bool transform_image(cv::Mat src, cv::Mat dst) {
 cv::Vec3b rgbpix; // OpenCV representation of an RGB pixel

 //set upper-left pixel to black
      Vec3b & color = g_virtual_image.at<Vec3b>(0,0);
        color[0] = 0;
        color[1] = 0;
        color[2] = 0;
        src.at<Vec3b>(0,0)=color;

 int u_cam,v_cam;
 for (int u_virt=0;u_virt<dst.cols;u_virt++) {
  for (int v_virt=0;v_virt<dst.rows;v_virt++) {
      u_cam= u_cam_mappings[u_virt][v_virt];
      v_cam = v_cam_mappings[u_virt][v_virt];
      dst.at<Vec3b>(v_virt, u_virt) = src.at<Vec3b>(v_cam,u_cam);
    }
  }
}


//class image_converter:
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  //void ImageConverterMouseCB(int event, int x, int y, int flags, void* userdata);

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe(g_input_image_topic.c_str(), 1,
      &ImageConverter::imageCb, this);

    image_pub_ = it_.advertise(g_output_image_topic.c_str(), 1);

    cv::namedWindow(OPENCV_WINDOW);
    //cv::namedWindow(OPENCV_ZOOM_WINDOW);
         //set the callback function for any mouse event
     //setMouseCallback(OPENCV_WINDOW, ImageConverterMouseCB, NULL);
     //setMouseCallback(OPENCV_WINDOW, ImageConverterMouseCB, NULL);
     //setMouseCallback(OPENCV_ZOOM_WINDOW, ImageConverterZoomedMouseCB, NULL);
     //    minimal_subscriber_ = nh_.subscribe("example_class_input_topic", 1, &ExampleRosClass::subscriberCallback,this);  

  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
    //cv::destroyWindow(OPENCV_ZOOM_WINDOW);
  }


//do transform inside this callback
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
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

    Mat dst_smaller;
    Mat src = cv_ptr->image;
    g_src = cv_ptr->image;


    if (g_first_time) {
      cout<<"first image received; cloning to transformed image"<<endl;
      g_virtual_image = g_src.clone();
      g_first_time=false;
      if (g_virtual_image.cols != g_image_width) {
          ROS_ERROR("image is not the expected size; exiting");
          exit(0);
      }
      if (g_virtual_image.rows != g_image_height) {
          ROS_ERROR("image is not the expected size; exiting");
          exit(0);
      }      
     cout<<"size height, width: "<<g_virtual_image.cols<<", "<<g_virtual_image.rows<<endl;
     cout<<"setting all pixels to black"<<endl;
     Vec3b & color = g_virtual_image.at<Vec3b>(0,0);
        color[0] = 0;
        color[1] = 0;
        color[2] = 0;
     for (int u=0;u<g_virtual_image.cols;u++) { //make sure correct order
       for (int v=0;v<g_virtual_image.rows;v++) {
         ////dst.at<cv::Vec3b>(u_virt, v_virt) = rgbpix;
         //Mat.at<data_Type>(row_num, col_num) = value;
         g_virtual_image.at<cv::Vec3b>(v,u) =   color; //CV_RGB(0,0,0);
       }
      }
     imshow(OPENCV_WINDOW, g_virtual_image);
     //cout<<"enter 1 to continue: ";
     //cin>>g_ans;
    cv::waitKey(3);

    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
    g_image_msg_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", g_virtual_image).toImageMsg();
    //image_pub_.publish(g_virtual_image);
    image_pub_.publish(g_image_msg_ptr);

    ROS_INFO("computing virtual to physical pixel mappings...");

    compute_mappings(g_affine_virt_cam_wrt_cam,g_image_xc,g_image_yc, g_image_fx, g_image_fy, VERT_CAM_HEIGHT,KPIX);

    ROS_INFO("done computing mappings");    
    
      return;
    }
    //ROS_INFO("transforming image...");
    transform_image(g_src,g_virtual_image);
    //cout<<"done transforming image"<<endl;
    resize(g_virtual_image, dst_smaller, Size(g_virtual_image.cols/RESCALE_FACTOR,g_virtual_image.rows/RESCALE_FACTOR));
    imshow(OPENCV_WINDOW, dst_smaller);  

    cv::waitKey(3);

    // publish transformed image

    g_image_msg_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", g_virtual_image).toImageMsg();
    //image_pub_.publish(g_virtual_image);
    image_pub_.publish(g_image_msg_ptr);

    //image_pub_.publish(cv_ptr->toImageMsg());
    g_got_new_image=true;
    //end of callback
  }
}; //end of class def








//subscribe to images;
//convert these to virtual images w/ ideal viewpoint and republish
int main(int argc, char** argv) {
  ros::init(argc, argv, "perspective_transform");
  ImageConverter ic;



 Eigen::Affine3d affine_cam_wrt_sys,affine_virt_cam_wrt_sys;
 // ImageConverter ic; //reuse imageConverter from interactive_image_gui2.cpp
 // this will receive images; use it to populate g_cam_image
 // have this fnc set g_got_new_image
  ros::NodeHandle nh;

 affine_cam_wrt_sys = get_hardcoded_affine_cam_wrt_sys();
 affine_virt_cam_wrt_sys = get_hardcoded_affine_virtual_cam_wrt_sys();
 g_affine_virt_cam_wrt_cam = affine_cam_wrt_sys.inverse()*affine_virt_cam_wrt_sys;
 
 

 //wait for first image
 ROS_INFO("waiting for camera_image publication...");
 while (!g_got_new_image) {
  ros::spinOnce();
  ros::Duration(1.0).sleep();
  std::cout<<".";
 }
 ROS_INFO("got first image; starting transform loop");

 //need an image publisher...find/copy this


 while (ros::ok()) {
   g_got_new_image=false;
  while (!g_got_new_image) {
    ros::spinOnce();
    ros::Duration(0.2).sleep();
   }
   //transform and publish image:
   //transform_image(cv::Mat src, cv::Mat dst, Eigen::Affined3d affine_virt_cam_wrt_cam, 
   // double cx_src,double cy_src, double fx_src, double fy_src, double vert_height, double kpix)
   // image_publisher.publish(g_virtual_image)
   ros::spinOnce();
   ros::Duration(0.5).sleep();

  }
 
}


