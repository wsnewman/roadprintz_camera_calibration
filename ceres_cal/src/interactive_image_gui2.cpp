//interactive_image_gui2.cpp
//displays topic /camera/image_rect_color
//can click mouse to get u,v coords
// will convert these to x,y w/rt sys_ref_frame
//in this version, try resizing in ROI's

//mouse-click modes:
//left-click in zoomed-out (whole-image) window --> create corresponding zoomed-in window;
// the zoomed window will initially refer to the first point, then to the second point, then alternate each time
// the whole-image window is left-clicked

//may left-click in zoomed window multiple times, then "accept" with right click

//after accepting first point, next zoomed window will imply 2nd point; 
//  right click on second point to accept 2nd point.

// if left-click on whole-image again, it will assume re-entry of first point, etc

// draw red dot (in both images) for first point and blue dot (in both images) for 2nd point
// draw a line (in whole-image view) connecting the first and second points

//if left-click in image window, sends command to EXECUTE (send polygon w/ 2 pts)

//if right-click in zoomed-out window-->EXECUTE: send the point pair (assuming there is a point pair specified)


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <iostream>
#include <fstream>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
//#include <xform_utils/xform_utils.h>

ros::Publisher g_pub_line;
const double RESCALE_FACTOR=2.0; //scale original image down by this factor to fit display size
const double ZOOM_FACTOR = 2.0; //scale up by this factor from original image to display zoom window
const double ROI_HEIGHT = 200.0; //choose size of ROI w/rt original image
const double ROI_WIDTH = 300.0;
double BOX_WIDTH = ROI_WIDTH/RESCALE_FACTOR;
double BOX_HEIGHT = ROI_HEIGHT/RESCALE_FACTOR;

//THESE magic numbers should get installed in robot_base to system_ref_frame transform
double HACK_SYS_REF_FRAME_X_OFFSET = 0.040;
double HACK_SYS_REF_FRAME_Y_OFFSET = -0.010;

//XformUtils xformUtils;
Eigen::Vector3d g_nom_surface_normal;
double g_nom_surface_plane_offset=0.0;

//MAGIC NUMBERS:
    //intrinsics:
double g_fx = 1637.367343; 
double g_fy = 1638.139550;   
double g_cx = 1313.144667;
double g_cy = 774.029343;

//also, currently using Eigen::Affine3d get_hardcoded_affine_cam_wrt_sys(void)
// need to make this more flexible; intend to implement via tf
//*** TODO
Eigen::Affine3d g_affine_cam_wrt_sys,g_affine_sys_wrt_cam;

static const std::string OPENCV_WINDOW = "ImageWindow";
static const std::string OPENCV_ZOOM_WINDOW = "ZoomWindow";

using namespace cv;
using namespace std;

std::vector<cv::Point> vertices;
cv::Point vertex;
ofstream paintfile;

Mat g_imCrop; //(ROI_WIDTH,ROI_HEIGHT); //global, so callback can affect it//cv::Mat test(cv::Size(1, 49)
Mat g_src;

double g_x_zoom_left=0;
double g_y_zoom_top=0;
double g_x_ctr=BOX_WIDTH/2;
double g_y_ctr=BOX_HEIGHT/2;
double g_zoom_x_ctr = ZOOM_FACTOR*ROI_WIDTH/2.0;
double g_zoom_y_ctr = ZOOM_FACTOR*ROI_HEIGHT/2.0;

bool g_first_time=true;

bool is_first_point=true;
geometry_msgs::Polygon g_endpoints;
geometry_msgs::Point32 start_pt,end_pt;
double g_u_pt1=0; //g_x_ctr;
double g_u_pt2=0; // = g_u_pt1;
double g_v_pt1=0;// g_y_ctr;
double g_v_pt2=0; // = g_v_pt1;

double g_u_zoomed_pt1=0; //=g_zoom_x_ctr;
double g_v_zoomed_pt1=0; //=g_zoom_y_ctr;
double g_u_zoomed_pt2=0; //g_u_zoomed_pt1;
double g_v_zoomed_pt2=0; //g_v_zoomed_pt1;

Rect2d g_r_zoom_window;

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

//inverse fnc, from (u,v) pixel values to (x,y,z) in camera frame is ambiguous, since cannot know the distance
//return a point that has unit z value, and x and y scaled appropriate to this value
//defer discovering of z-height
void pixel_to_pt_wrt_cam_frame(double u, double v, Eigen::Vector3d &pt_wrt_cam) {
    pt_wrt_cam[2] = 1.0; //definitional for this function
    pt_wrt_cam[0] = (u-g_cx)/g_fx;
    pt_wrt_cam[1] = (v-g_cy)/g_fy;    
}

//more sophisticated fnc to convert pixel values to 3-D coords w/rt sys_ref_frame:
void convert_pixel_to_meters(double u_val,double v_val,Eigen::Vector3d surface_plane_normal,double surface_plane_offset,
        Eigen::Vector3d &pt_wrt_sys_ref_frame) {
    //ROS_INFO("input pixel values (u,v) =  (%f, %f), angle %f",u_val,v_val,theta_sym_wrt_sys);
    Eigen::Vector3d vec_wrt_camera_frame, O_cam_wrt_sys_ref_frame,vec_wrt_sys_ref_frame;
    //convert the (u,v) point to camera_frame coords, with p_z = 1.0, 
    //g_affine_cam_wrt_sys
    pixel_to_pt_wrt_cam_frame(u_val, v_val, vec_wrt_camera_frame);
    //ROS_INFO("vec_wrt_camera_frame = %f, %f, %f",vec_wrt_camera_frame[0],vec_wrt_camera_frame[1],vec_wrt_camera_frame[2]);

    //convert this to sys_ref_frame:
    vec_wrt_sys_ref_frame = g_affine_cam_wrt_sys.linear()*vec_wrt_camera_frame;
    //ROS_INFO("vec_wrt_sys_ref_frame = %f, %f, %f",vec_wrt_sys_ref_frame[0],vec_wrt_sys_ref_frame[1],vec_wrt_sys_ref_frame[2]);
    
    //solve for intersection w/ specified plane:
    ROS_WARN("convert_pixel_to_meters is assuming ideal plane");
    //double z_height = pt_wrt_sys_ref_frame[2];
    //rescale pt s.t. the height in sys_ref_frame = 0
    O_cam_wrt_sys_ref_frame = g_affine_cam_wrt_sys.translation();
    //ROS_INFO("O_cam_wrt_sys_ref_frame = %f, %f, %f",O_cam_wrt_sys_ref_frame[0],O_cam_wrt_sys_ref_frame[1],O_cam_wrt_sys_ref_frame[2]);
    double v_length = -O_cam_wrt_sys_ref_frame[2]/vec_wrt_sys_ref_frame[2];
    pt_wrt_sys_ref_frame = O_cam_wrt_sys_ref_frame + v_length*vec_wrt_sys_ref_frame;
    
    pt_wrt_sys_ref_frame[0] = pt_wrt_sys_ref_frame[0] + HACK_SYS_REF_FRAME_X_OFFSET;
    pt_wrt_sys_ref_frame[1] = pt_wrt_sys_ref_frame[1] + HACK_SYS_REF_FRAME_Y_OFFSET;
    
}




  

void ImageConverterMouseCB(int event, int x, int y, int flags, void* userdata)
{
    //float x_sys,y_sys;

    if  ( event == EVENT_LBUTTONDOWN )
    {
        g_x_ctr = x;
        g_y_ctr = y;
        
        //require that zoom will fit box:
        //Point2f upper_left((g_x_ctr-box_width/2),(g_y_ctr-box_height/2));
        //Point2f lower_right((g_x_ctr+box_width/2),(g_y_ctr+box_height/2));
        if (g_x_ctr<BOX_WIDTH/2) {
            g_x_ctr = BOX_WIDTH/2;
        }
        if (g_y_ctr<BOX_HEIGHT/2) {
            g_y_ctr = BOX_HEIGHT/2;
        }        
        
        //test for ctr coords too large:
        //    resize(g_src, src_smaller, Size(g_src.cols/RESCALE_FACTOR,g_src.rows/RESCALE_FACTOR));

        if (g_x_ctr>g_src.cols/RESCALE_FACTOR-BOX_WIDTH/2-1) {
            g_x_ctr = g_src.cols/RESCALE_FACTOR-BOX_WIDTH/2-1;
        }
        if (g_y_ctr>g_src.rows/RESCALE_FACTOR-BOX_WIDTH/2-1) {
            g_y_ctr = g_src.rows/RESCALE_FACTOR-BOX_HEIGHT/2-1;
        }            
             
        // Select ROI; zoom display
        //make the x,y click be the center of the ROI,
        //and use a fixed size for zoom
        //Rect2d r; // = selectROI(src_smaller);
        double x_zoom_ctr,y_zoom_ctr;

        x_zoom_ctr = RESCALE_FACTOR*x; 
        g_x_zoom_left = x_zoom_ctr - ROI_WIDTH/2;
        if (g_x_zoom_left<0) g_x_zoom_left=0;
        if (g_x_zoom_left+ROI_WIDTH>=g_src.cols) {
            g_x_zoom_left = g_src.cols-ROI_WIDTH-1;
        }
        g_r_zoom_window.x = g_x_zoom_left;
        
        y_zoom_ctr = RESCALE_FACTOR*y; 
        g_y_zoom_top = y_zoom_ctr - ROI_HEIGHT/2;
        if (g_y_zoom_top<0) g_y_zoom_top=0;
        if (g_y_zoom_top+ROI_HEIGHT>=g_src.rows) {
            g_y_zoom_top = g_src.rows-ROI_HEIGHT-1;
        }
        g_r_zoom_window.y = g_y_zoom_top;        
        

        
        
        g_r_zoom_window.height = ROI_HEIGHT;
        g_r_zoom_window.width = ROI_WIDTH;
        //g_imCrop = g_src(g_r_zoom_window);    // Crop image   
        //resize(g_imCrop, g_imCrop, Size(ROI_WIDTH*ZOOM_FACTOR,ROI_HEIGHT*ZOOM_FACTOR));
    }
        //toggle the current point
        //is_first_point =!is_first_point;
    if  ( event == EVENT_RBUTTONDOWN )
    {
        ROS_INFO("publishing polygon: "); 
        ROS_INFO("start pt = %f, %f",g_endpoints.points[0].x,g_endpoints.points[0].y);
        ROS_INFO("end pt = %f, %f",g_endpoints.points[1].x,g_endpoints.points[1].y);
        g_pub_line.publish(g_endpoints); 
    }
}

void ImageConverterZoomedMouseCB(int event, int x, int y, int flags, void* userdata)
{

    if  ( event == EVENT_LBUTTONDOWN )
    {
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        //convert this back to original image coords, u_val, v_val
        //x,y is distance from upper left corner;
        //but upper left corner is at 
        //g_u_zoomed_pt1
        double u_val = x/ZOOM_FACTOR+g_x_zoom_left;
        double v_val = y/ZOOM_FACTOR+g_y_zoom_top;
        cout<< "rescaled: "<< u_val<<","<<v_val<<endl;
        Eigen::Vector3d pt_wrt_sys_ref_frame;
        
        convert_pixel_to_meters( u_val, v_val, g_nom_surface_normal,g_nom_surface_plane_offset,pt_wrt_sys_ref_frame);
        
        //g_u_zoomed_pt1
        if (is_first_point) {
            cout<<"setting starting point: ";
            g_u_zoomed_pt1 = x; //u_val;
            g_v_zoomed_pt1 = y; //v_val;
            start_pt.x = (float) pt_wrt_sys_ref_frame[0];
            start_pt.y = (float) pt_wrt_sys_ref_frame[1];
            g_endpoints.points[0]=start_pt;
            //convert to coords in whole-image view:
            g_u_pt1 =  u_val/RESCALE_FACTOR; //(ZOOM_FACTOR*RESCALE_FACTOR);
            g_v_pt1 =  v_val/RESCALE_FACTOR; //(ZOOM_FACTOR*RESCALE_FACTOR);
            
        }

        else
        {
            cout<<"setting end point"<<endl;
            g_u_zoomed_pt2 = x; //u_val;
            g_v_zoomed_pt2 = y; //v_val;
            end_pt.x = (float) pt_wrt_sys_ref_frame[0];
            end_pt.y = (float) pt_wrt_sys_ref_frame[1];
            g_endpoints.points[1]=end_pt;
            g_u_pt2 =  u_val/RESCALE_FACTOR;
            g_v_pt2 =  v_val/RESCALE_FACTOR;            
        }
        /*
        geometry_msgs::Point des_point_wrt_sys_ref_frame;
        //populate this
        des_point_wrt_sys_ref_frame.x=pt_wrt_sys_ref_frame[0];
        des_point_wrt_sys_ref_frame.y=pt_wrt_sys_ref_frame[1];
        des_point_wrt_sys_ref_frame.z=0.0;
        
      
        //publish this:
        g_pub_line.publish(des_point_wrt_sys_ref_frame); */
    }
    if  ( event == EVENT_RBUTTONDOWN )
    {
       
        if (is_first_point) {
            cout<<"changing to end point entry"<<endl;
        }
        else  {
              cout<<"changing to start point entry"<<endl;
        }        
        is_first_point = !is_first_point; 
    }
 
}


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
    image_sub_ = it_.subscribe("/camera/image_rect_color", 1,
      &ImageConverter::imageCb, this);
    //image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
    cv::namedWindow(OPENCV_ZOOM_WINDOW);
         //set the callback function for any mouse event
     //setMouseCallback(OPENCV_WINDOW, ImageConverterMouseCB, NULL);
     setMouseCallback(OPENCV_WINDOW, ImageConverterMouseCB, NULL);
     setMouseCallback(OPENCV_ZOOM_WINDOW, ImageConverterZoomedMouseCB, NULL);
     //    minimal_subscriber_ = nh_.subscribe("example_class_input_topic", 1, &ExampleRosClass::subscriberCallback,this);  

  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
    cv::destroyWindow(OPENCV_ZOOM_WINDOW);
  }

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
    Mat src_smaller;
    //Mat src = cv_ptr->image;
    g_src = cv_ptr->image;
    resize(g_src, src_smaller, Size(g_src.cols/RESCALE_FACTOR,g_src.rows/RESCALE_FACTOR));

    //Mat imCrop = src;

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
 
    
    
    imshow(OPENCV_WINDOW, src_smaller);  

   // Crop image   
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
    
    imshow(OPENCV_ZOOM_WINDOW,g_imCrop);    
    
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);

    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

  //void ImageConverterMouseCB(int event, int x, int y, int flags, void* userdata);




int main(int argc, char** argv)
{
  ros::init(argc, argv, "interactive_image_gui");
  ImageConverter ic;
  ros::NodeHandle nh;
  ros::Publisher pub_line =
    nh.advertise<geometry_msgs::Polygon>("polygon_wrt_sys_ref_frame", 1, true);

    // make this publisher available to callback
    g_pub_line = pub_line;
  g_affine_cam_wrt_sys = get_hardcoded_affine_cam_wrt_sys();
  g_nom_surface_normal<<0,0,1; //default ideal plane
  //paintfile.open("interactive_paintfile.cpf",ios::out|ios::trunc);
  //vertices.clear();
  g_endpoints.points.resize(2);
  start_pt.x=0;
  start_pt.y=0;
  start_pt.z=0;
  end_pt.x=0.0;
  end_pt.y=0.0;
  end_pt.z=0.0;
  g_endpoints.points[0]=start_pt;
  g_endpoints.points[1]=end_pt;
  
  
  ros::spin();
  return 0;
}
