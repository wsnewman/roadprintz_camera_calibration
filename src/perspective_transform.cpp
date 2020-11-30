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
const double g_image_width = 2688;
const double g_image_height = 1520;  
const double   g_image_xc = 1313.144667; //do these change  for rectified image?
const double   g_image_yc = 774.029343;
const double g_image_fx = 1637.367343;
const double g_image_fy = 1638.139550;

const double g_virt_image_width = 2688; //might want to change these if change resolution of virtual image
const double g_virt_image_height = 1520; 



const double VERT_CAM_HEIGHT=3.0; //probably don't need this
const double KPIX = 590.0; //pixels per meter for virtual image; might want to change, e.g., to 1000.0
//const double CX_CAM = xxx;
//const double CY_CAM = xxx;
static const std::string OPENCV_WINDOW = "ImageWindow";

Eigen::Affine3d g_affine_virt_cam_wrt_cam;


cv::Mat g_virtual_image; //(Nu,Nv,CV_8U,cv::Scalar(0));
cv::Mat g_src;  // need access to this; subsribe?
sensor_msgs::ImagePtr g_image_msg_ptr;

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
        trans<<  2.76911, 0.00777921,    2.88495; //4.44
    //note: camera origin is about 2.885 meters high, and about 2.768m aft of rear wheels
    //   camera origin is nearly along vehicle centerline, but shifted about 7mm to starboard

     R <<     0.0525021,  0.995724,   0.0759998,
              0.9976,    -0.055738,   0.0410976,
              0.0451579,  0.0736596, -0.996261;
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
        trans<<  2.77, 0.0,    2.885; //
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

//set camera image as src; set transformed image to be created as dst
//specify central pixel of input image, (cx_src,cy_src), and foci of input impage, (fx_src,fy_src)
//specify transform of virtual cam frame w/rt actual cam frame
//specify height of ideal, virtual image to be populated
//specify scaling of new iage: kpix = pixels/meter at ideal plane, from height vert_height
//create new, virtual (dst) image transformed from src image
bool transform_image(cv::Mat src, cv::Mat dst, Eigen::Affine3d affine_virt_cam_wrt_cam, 
  double cx_src,double cy_src, double fx_src, double fy_src, double vert_height, double kpix) {
 cv::Vec3b rgbpix; // OpenCV representation of an RGB pixel


 int Nu_virt = dst.cols;
 int Nv_virt = dst.rows;
 int Nu_cam = src.cols;
 int Nv_cam = src.rows;
 double cx_virt = 0.5*Nu_virt;  //virtual image has ideal central pixel
 double cy_virt = 0.5*Nv_virt; 
 Eigen::Vector3d p_wrt_virt,p_wrt_cam;
  cout<<"transforming image; cx_virt = "<<cx_virt<<"; cy_virt = "<<cy_virt<<", Nu_virt = "<<Nu_virt<<", Nv_virt = "<<Nv_virt<<endl;

 //iterate through rows and columns of image to be created (dst)


 int u_cam,v_cam;
 p_wrt_virt[2] = vert_height;
 for (int u_virt=0;u_virt<dst.cols;u_virt++) {
  for (int v_virt=0;v_virt<dst.rows;v_virt++) {
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
    if (valid_uv(u_cam,v_cam,Nu_cam,Nv_cam)) {
      //cout<<"valid u_cam, v_cam = "<<u_cam<<", "<<v_cam<<endl;
       //Mat.at<data_Type>(row_num, col_num) = value;
      dst.at<Vec3b>(v_virt, u_virt) = src.at<Vec3b>(v_cam,u_cam);
      //rgbpix = src.at<cv::Vec3b>(u_cam, v_cam); //extract an RGB pixel
      //dst.at<cv::Vec3b>(u_virt, v_virt) = rgbpix;
    }
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

    /*
    if (g_first_time) { //one-shot: is a hack
        Rect2d r;
        g_src = cv_ptr->image;
        r.x = g_x_zoom_left;
        r.y = g_y_zoom_top;         
        r.height = ROI_HEIGHT;
        r.width = ROI_WIDTH;
        g_r_zoom_window = r;
        g_imCrop = g_src(r);    // Crop image   
        resize(g_imCrop, g_imCrop, Size(ROI_WIDTH*ZOOM_FACTOR,ROI_HEIGHT*ZOOM_FACTOR));
        g_first_time=false;
    }
    */
    // Draw an example circle on the video stream
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    /*
    int n_vertices = vertices.size();
    if (n_vertices>1) {
        for (int i=0;i<n_vertices-1;i++) {
            if(spray_on_vec[i]) {
                line(cv_ptr->image,vertices[i],vertices[i+1],cv::Scalar( 255, 0, 0 ),8);
            }
        }
    } */
    // Update GUI Window
    //cout<<"calling imshow"<<endl;
    //Mat src_smaller;
    Mat dst_smaller;
    Mat src = cv_ptr->image;
    g_src = cv_ptr->image;


    //Mat imCrop = src;

   /*
    // Display Cropped Image
    cv::Scalar blue( 255, 0, 0 );
    cv::Scalar red( 0, 0, 255 );
    cv::Scalar green(0,255,0);
    cv::circle(src_smaller, cv::Point(g_x_ctr, g_y_ctr), 10, green,2);
    //show endpts and line on whole-view image    
    cv::circle(src_smaller, cv::Point(g_u_pt1, g_v_pt1), 2, red,2);
    cv::circle(src_smaller, cv::Point(g_u_pt2, g_v_pt2), 2, blue,2);
    cv::line(src_smaller,cv::Point(g_u_pt1, g_v_pt1),cv::Point(g_u_pt2, g_v_pt2),green,1,cv::LINE_AA);



    //double box_x_ctr = g_x_ctr
    //cv::rectangle( img, cv::Point2f( 10, 10 ), cv::Point2f(100, 100), cv::Scalar( 255, 0, 0 ) );
    Point2f upper_left((g_x_ctr-BOX_WIDTH/2),(g_y_ctr-BOX_HEIGHT/2));
    Point2f lower_right((g_x_ctr+BOX_WIDTH/2),(g_y_ctr+BOX_HEIGHT/2));
    cv::Scalar color( 255, 0, 0 );
    if (is_first_point) {
      cv::rectangle(src_smaller,upper_left,lower_right,red, 2);
    }
    else {
        cv::rectangle(src_smaller,upper_left,lower_right,blue, 2);
    }
 
    */


/* bool transform_image(cv::Mat src, cv::Mat dst, Eigen::Affine3d affine_virt_cam_wrt_cam, 
  double cx_src,double cy_src, double fx_src, double fy_src, double vert_height, double kpix)
*/


    if (g_first_time) {
      cout<<"first image received; cloning to transformed image"<<endl;
       g_virtual_image = g_src.clone();
      g_first_time=false;

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
      return;
    }

    transform_image(g_src,g_virtual_image,g_affine_virt_cam_wrt_cam,g_image_xc,g_image_yc, g_image_fx, g_image_fy, VERT_CAM_HEIGHT,KPIX);
    cout<<"done transforming image"<<endl;
    resize(g_virtual_image, dst_smaller, Size(g_virtual_image.cols/RESCALE_FACTOR,g_virtual_image.rows/RESCALE_FACTOR));
    imshow(OPENCV_WINDOW, dst_smaller);  
    //imshow(OPENCV_WINDOW, g_src);  

   // Crop image  
   /* 
    g_imCrop = g_src(g_r_zoom_window);
    resize(g_imCrop, g_imCrop, Size(ROI_WIDTH*ZOOM_FACTOR,ROI_HEIGHT*ZOOM_FACTOR));
    cv::circle(g_imCrop, cv::Point(g_zoom_x_ctr, g_zoom_y_ctr), 20, green,2);
    if (is_first_point) {
       cv::rectangle(g_imCrop, cv::Point(0,0), cv::Point(ZOOM_FACTOR*ROI_WIDTH,ZOOM_FACTOR*ROI_HEIGHT), red,2);
        
       cv::circle(g_imCrop, cv::Point(g_u_zoomed_pt1, g_v_zoomed_pt1), 2, red,2);
    }
    else {
       cv::rectangle(g_imCrop, cv::Point(0,0), cv::Point(ZOOM_FACTOR*ROI_WIDTH,ZOOM_FACTOR*ROI_HEIGHT), blue,2);
        
     cv::circle(g_imCrop, cv::Point(g_u_zoomed_pt2, g_v_zoomed_pt2), 2, blue,2);
    }
    
    imshow(OPENCV_ZOOM_WINDOW,g_imCrop);    */
    
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);

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


