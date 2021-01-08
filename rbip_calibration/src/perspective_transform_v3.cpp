//v2: 
//program to convert an input image into a top-down image, normal to ideal plane and aligned w/ x-y axes
//extend to projections based on LIDAR data...lots of work



//include correction for surface-map height; could be very slow
// rqrs knowledge of input image extrinsics in terms of an affine transform

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_generator/MapperSrv.h>


#include <iostream>
#include <fstream>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <xform_utils/xform_utils.h>
XformUtils xform_utils;

#include <camera_parameters/virtual_cam_params.h>
#include <camera_parameters/stella_amcrest_ipcam_params.h>
#include <camera_parameters/CamCalib.h>

int g_image_xc = g_rect_width/2;
int g_image_yc = g_rect_height/2;
/*
const double g_virt_image_width = 3000; //2688; //might want to change these if change resolution of virtual image
const double g_virt_image_height = 2000; //1520; 


//experiment with values to get desired pixel scaling, pix/meter
const double VERT_CAM_HEIGHT=3.0; //this actually makes NO difference; but need some value here 
const double VIRT_CAM_X_OFFSET=1.45; //choose this to be near the physical camera offset

const double KPIX = 590.0; //pixels per meter for virtual image; might want to change, e.g., to 1000.0
*/

using namespace cv;
using namespace std;

const double RESCALE_FACTOR=2.0; //scale original image down by this factor to fit display size


//convert camera image into an ideal image, as viewed normal to the ideal plane
//SHOULD get the following values from camera_info...fix later


//numbers for Stella's camera:
/*
const int g_image_width = 2688;
const int g_image_height = 1520;  
const double   g_image_xc = 1313.144667; //do these change  for rectified image?  I think not
const double   g_image_yc = 774.029343;
const double g_image_fx = 1637.367343;
const double g_image_fy = 1638.139550;
*/


static const std::string OPENCV_WINDOW = "ImageWindow";

Eigen::Affine3d g_affine_virt_cam_wrt_cam;

//global pointer for surface map service:
ros::ServiceClient *g_surface_map_service_client_ptr;
pcl_generator::MapperSrv g_map_srv;


cv::Mat g_virtual_image; //(Nu,Nv,CV_8U,cv::Scalar(0));
cv::Mat g_src;  // need access to this; subsribe?
sensor_msgs::ImagePtr g_image_msg_ptr;

//precompute how u_virt,v_virt map ont u_cam,v_cam
//hack: assign black to corner (0,0) in physical image, and
// for all (u_virt,v_virt) coordinates that do not have a correspondence in physical image,
// assign u_cam,v_cam = 0,0
int u_cam_mappings[g_virt_image_width][g_virt_image_height];
int v_cam_mappings[g_virt_image_width][g_virt_image_height];


bool g_got_new_image = false;
bool g_first_time = true;
bool g_got_transforms = false;

int g_ans;

//issue here: topics to subscribe and publish
//HMI expects "/camera/image_rect_color" (actually, compressed version)
//but want to pipe this through this perspective transform first
std::string g_output_image_topic="/camera/image_rect_color";
std::string g_input_image_topic="/camera_pretransformed/image_rect_color";

//SHOULD get this from tf publication; hardcode test
//vals from 12/30/20 calibration data w/ Stella
//this is used to convert affine to pose;
//then can enter pose data in launch file: roadprintz_launch/stella_camera_optical_frame_12_30_20_calib.launch
Eigen::Affine3d eval_hardcoded_affine_cam_wrt_RBIP(void) {
	Eigen::Affine3d test_affine_cam_wrt_RBIP;

	Eigen::Vector3d trans;
	Eigen::Matrix3d R;
    trans<<1.44604, -0.00440006, 2.86339;//0.918912 pixels RMS reprojection error
    R<<0.0561968, 0.995697, 0.0736899,
    0.997359, -0.0593846, 0.0418105,
    0.0460069, 0.0711456, -0.996403;    

	test_affine_cam_wrt_RBIP.linear() = R;
	test_affine_cam_wrt_RBIP.translation() = trans;
        
        //tf::StampedTransform tfCamWrtRBIP;
        geometry_msgs::Pose poseCamWrtRBIP;
        
        poseCamWrtRBIP = xform_utils.transformEigenAffine3dToPose(test_affine_cam_wrt_RBIP);
        ROS_INFO("from hardcoded cam wrt RBIP: test_affine_cam_wrt_RBIP");
        xform_utils.printAffine(test_affine_cam_wrt_RBIP);        
        ROS_INFO("equivalent pose: ");
        xform_utils.printPose(poseCamWrtRBIP);
	return test_affine_cam_wrt_RBIP;

} 


//define an ideal transform for the virtual camera;
//may choose origin to be close to physical camera origin, so field of view is comparable
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
    double cx_virt = g_virt_image_width/2.0;
    double cy_virt = g_image_height/2.0;
    Eigen::Vector3d p_wrt_virt,p_wrt_cam;
  int u_cam,v_cam;
  p_wrt_virt[2] = vert_height;
 for (int u_virt=0;u_virt<g_virt_image_width;u_virt++) {
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
    
    //validity w/rt dimensions of pre-transformed image:
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

    if (!g_got_transforms) {
        ROS_WARN("got image, but transforms not ready yet");
        return;
    }
    
    if (g_first_time) {
      cout<<"first image received; cloning to transformed image"<<endl;
      g_virtual_image = g_src.clone();
      //cv::resize(img, img_dst, cv::Size(640, 480), 0, 0, cv::INTER_AREA);
      resize(g_src,g_virtual_image,Size(g_virt_image_width,g_virt_image_height),0,0,cv::INTER_AREA);
      g_first_time=false;
      if (g_src.cols != g_rect_width) {
          ROS_ERROR("image is not the expected size; exiting");
          exit(0);
      }
      if (g_src.rows != g_rect_height) {
          ROS_ERROR("image is not the expected size; exiting");
          exit(0);
      }      
     cout<<"input image size height, width: "<<g_src.cols<<", "<<g_src.rows<<endl;
     cout<<"setting all pixels of virtual (perspective dewarped) image to black"<<endl;
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

    //ugh.  maybe the virtual and real camera parameters headers should live in their own package!
    
    compute_mappings(g_affine_virt_cam_wrt_cam,g_image_xc,g_image_yc, g_fx, g_fy, VERT_CAM_HEIGHT,KPIX);

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



bool cam_calib_callback(camera_parameters::CamCalibRequest &request, camera_parameters::CamCalibResponse &response)
{
    ROS_INFO("cam_calib_data_service callback activated....");
    ROS_INFO("responding with Amcrest (Stella) cam extrinsic calibration data");

        response.i0 = 0; //no longer used
        response.j0 = 0; //no longer used
        response.pix_per_meter = KPIX; //reconcile this with perspective_transform node; per header file
        //response.camera_name = ip_cam::CamCalibResponse::AMCREST_CAM; //STELLA_AMCREST; should no longer be relevant
        response.success=true;
        return true;
}




//subscribe to images;
//convert these to virtual images w/ ideal viewpoint and republish
int main(int argc, char** argv) {
  ros::init(argc, argv, "perspective_transform_v2");
  ImageConverter ic;



 Eigen::Affine3d affine_cam_wrt_RBIP,affine_virt_cam_wrt_RBIP;
 // ImageConverter ic; //reuse imageConverter from interactive_image_gui2.cpp
 // this will receive images; use it to populate g_cam_image
 // have this fnc set g_got_new_image
  ros::NodeHandle nh;
  
      tf::TransformListener tfListener;
    bool tferr = true;
    ROS_INFO("waiting for tf between RBIP frame and robot's base_link...");
    tf::StampedTransform tfRBIPwrtBaseLink; //tfBaseLinkWrtTruck;
    while (tferr) {
        tferr = false;
        try {
            //try to lookup transform, link2-frame w/rt base_link frame; this will test if
            // a valid transform chain has been published from base_frame to link2
            tfListener.lookupTransform("base_link", "RBIP_frame", ros::Time(0), tfRBIPwrtBaseLink);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good, base_link to RBIP frame");


    Eigen::Affine3d affine_RBIP_wrt_base_link = xform_utils.transformStampedTfToEigenAffine3d(tfRBIPwrtBaseLink);
    //xform_utils.printAffine(affine_RBIP_wrt_base_link);
    Eigen::Affine3d affine_base_link_wrt_RBIP;
    affine_base_link_wrt_RBIP = affine_RBIP_wrt_base_link.inverse();
    ROS_INFO("affine_base_link_wrt_RBIP");
    xform_utils.printAffine(affine_base_link_wrt_RBIP);

       eval_hardcoded_affine_cam_wrt_RBIP();

       
    //get the physical camera frame w/rt RBIP frame--corresponding to robot in camera pose    
    tferr = true;
    ROS_INFO("waiting for tf between RBIP frame and physical camera optical frame...");
    tf::StampedTransform tfCamWrtRBIP; 
    while (tferr) {
        tferr = false;
        try {
            //try to lookup transform, link2-frame w/rt base_link frame; this will test if
            // a valid transform chain has been published from base_frame to link2
            tfListener.lookupTransform("RBIP_frame","camera_optical_frame" , ros::Time(0), tfCamWrtRBIP);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good, RBIP frame and camera optical frame");
    affine_cam_wrt_RBIP = xform_utils.transformStampedTfToEigenAffine3d(tfCamWrtRBIP);
    ROS_INFO("affine_cam_wrt_RBIP");
    xform_utils.printAffine(affine_cam_wrt_RBIP);  
   
  
 //affine_cam_wrt_RBIP = get_hardcoded_affine_cam_wrt_sys();
 affine_virt_cam_wrt_RBIP = get_hardcoded_affine_virtual_cam_wrt_RBIP();
 g_affine_virt_cam_wrt_cam = affine_cam_wrt_RBIP.inverse()*affine_virt_cam_wrt_RBIP;
  g_got_transforms=true;

  //the only thing this does is informs the HMI of the value of KPIX, which is in the virtual-camera header file
  ros::ServiceServer calib_service = nh.advertiseService("cam_calib_data_service", cam_calib_callback);

      
      
  ros::ServiceClient client = nh.serviceClient<pcl_generator::MapperSrv>("surface_map_service");
  g_surface_map_service_client_ptr = &client;
  //test the surface mapper service:
  
 
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


