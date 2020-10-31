//interactive_image_gui.cpp
//displays topic /camera/image_rect_color
//can click mouse to get u,v coords
// will convert these to x,y w/rt sys_ref_frame

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <geometry_msgs/Point.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
//#include <xform_utils/xform_utils.h>

ros::Publisher g_pub_point;

const double X_OFFSET_MANUAL_HACK = 0.040; //laser is shifted this much fore rel to target
const double Y_OFFSET_MANUAL_HACK = -0.010; //laser is shifted this much to starboard rel to target

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

     double   fx = 1637.367343; 
     double   fy = 1638.139550;   
     //double f = 1638.0; // can't tell which is which
     //I get a better fit swapping cx and cy:
     double ci = 774.029343;  //hmm...I hope these are the right order!
     double cj = 1313.144667; //try swapping?  OR...swap input values of image coords
     double cx = cj;
     double cy = ci;

static const std::string OPENCV_WINDOW = "ImageWindow";
using namespace cv;
using namespace std;

std::vector<cv::Point> vertices;
cv::Point vertex;
ofstream paintfile;
double arrival_time=0;

double rescale_factor=1.5;

//xxxxxxxx      HARD-CODED calibration vals; should put these elsewhere...header, param server...
float dxdi= -1.683; //mm/pixel
float dydj= 1.723; //mm/pixel
float Ox_ref = 3.0; //measure from here, reference marker at (x,y)/sys = (3.0,0)
float Oy_ref = 0.0;
int i_ref = 1140; // i-pixel onto which (3,0) projects
int j_ref = 574;  // j-pixel onto which (3,0) projects    
static int Ncols_logitech = 1920;
static int Nrows_logitech = 1080;
float sprayhead_vertical_offset= 0.1; //set height of sprayhead; ADJUST ME
float paint_speed= 0.1; //paint speed, m/s; ADJUST ME
float x_sys_prev,y_sys_prev;
bool first_point=true;
int sprayhead_on=false;
std::vector<int> spray_on_vec;

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
}




bool hack_ij_from_xy(float Ox_ref, float Oy_ref, int i_ref, int j_ref, float dxdi, float dydj, float x_des, float y_des, int &i_val, int &j_val) {
    Eigen::Matrix2f scale_mat;
    Eigen::Vector2f col0,col1;
    col0<<dxdi/1000.0,0.0; //convert to m/pixel
    col1<<0.0,dydj/1000.0;
    scale_mat.col(0) = col0;
    scale_mat.col(1) = col1;
    //scale_mat[0][0] = dxdi;
    //scale_mat[0][1] = 0.0;
    //scale_mat[1][0] = 0.0;
    //scale_mat[1][1] = dydj;
    Eigen::Vector2f ij_ref,ij_des,xy_ref,xy_des;
    ij_ref<<i_ref,j_ref;
    xy_ref<<Ox_ref,Oy_ref;
    xy_des<<x_des,y_des;
    //xy_des = scale_mat*(ij_des-ij_ref) + xy_ref;
    //inv(scale_mat)*(xy_des-xy_ref)+ij_ref = ij_des;  test if result is valid; return values and bool
    ij_des = scale_mat.inverse()*(xy_des-xy_ref) + ij_ref;
    i_val = ij_des[0];
    j_val = ij_des[1];
    if ((i_val >= 0)&& (i_val < Ncols_logitech) &&(j_val >= 0)&&(j_val < Nrows_logitech)) { return true; }
            else {
                return false;
            }
    
}

//go the othe way: i,j--> x,y
bool hack_xy_from_ij(float Ox_ref, float Oy_ref, int i_ref, int j_ref, int i_val, int j_val, float dxdi, float dydj, float &x_des, float &y_des) {
    Eigen::Matrix2f scale_mat;
    Eigen::Vector2f col0,col1;
    col0<<dxdi/1000.0,0.0; //convert to m/pixel
    col1<<0.0,dydj/1000.0;
    scale_mat.col(0) = col0;
    scale_mat.col(1) = col1;
    //scale_mat[0][0] = dxdi;
    //scale_mat[0][1] = 0.0;
    //scale_mat[1][0] = 0.0;
    //scale_mat[1][1] = dydj;
    Eigen::Vector2f ij_ref,ij_des,xy_ref,xy_des;
    ij_des<<i_val,j_val;
    ij_ref<<i_ref,j_ref;
    xy_ref<<Ox_ref,Oy_ref;
    xy_des<<x_des,y_des;
    xy_des = scale_mat*(ij_des-ij_ref) + xy_ref;
    ROS_INFO_STREAM("ij_ref = "<<ij_ref.transpose()<<endl);
    ROS_INFO_STREAM("ij_des = "<<ij_des.transpose()<<endl);
    ROS_INFO_STREAM("xy_ref = "<<xy_ref.transpose()<<endl);
    ROS_INFO_STREAM("xy_des = "<<xy_des.transpose()<<endl);
    
    x_des = xy_des[0];
    y_des = xy_des[1];
    //inv(scale_mat)*(xy_des-xy_ref)+ij_ref = ij_des;  test if result is valid; return values and bool
    //ij_des = scale_mat.inverse()*(xy_des-xy_ref) + ij_ref;
    //i_val = ij_des[0];
    //j_val = ij_des[1];
    if ((i_val >= 0)&& (i_val < Ncols_logitech) &&(j_val >= 0)&&(j_val < Nrows_logitech)) { return true; }
            else {
                return false;
            }
    
}
  

void ImageConverterMouseCB(int event, int x, int y, int flags, void* userdata)
{
    float x_sys,y_sys;
    /*
    hack_xy_from_ij(Ox_ref, Oy_ref, i_ref,  j_ref, x, y, dxdi, dydj, x_sys, y_sys);
    ROS_INFO("x_sys,y_sys = %f, %f",x_sys,y_sys);
    //compute arrival time based on distance from previous point to new point
    if(first_point) { 
        arrival_time=0;
        first_point=false;
        ROS_INFO("first point: arrival time set to zero");
    }
    else {
        double dx,dy,move_time;
        dx = x_sys-x_sys_prev;
        dy = y_sys-y_sys_prev;
        move_time = sqrt(dx*dx+dy*dy)/paint_speed;
        ROS_INFO("computed move time = %f",move_time);
        arrival_time+=move_time;
        }
     * */
    if  ( event == EVENT_LBUTTONDOWN )
    {
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        double u_val = rescale_factor*x;
        double v_val = rescale_factor*y;
        cout<< "rescaled: "<< u_val<<","<<v_val<<endl;
        Eigen::Vector3d pt_wrt_sys_ref_frame;
        
        convert_pixel_to_meters( u_val, v_val, g_nom_surface_normal,g_nom_surface_plane_offset,pt_wrt_sys_ref_frame);
        
        geometry_msgs::Point des_point_wrt_sys_ref_frame;
        //populate this
        ROS_WARN("using offsets %f, %f",X_OFFSET_MANUAL_HACK,Y_OFFSET_MANUAL_HACK);
        des_point_wrt_sys_ref_frame.x=pt_wrt_sys_ref_frame[0]+X_OFFSET_MANUAL_HACK;
        des_point_wrt_sys_ref_frame.y=pt_wrt_sys_ref_frame[1]+Y_OFFSET_MANUAL_HACK;
        des_point_wrt_sys_ref_frame.z=0.0;  
        
        
        
        //publish this:
        g_pub_point.publish(des_point_wrt_sys_ref_frame);
        
        /* vertex.x = x;
        vertex.y = y;
        vertices.push_back(vertex);

        sprayhead_on=true; 
        spray_on_vec.push_back(sprayhead_on);

    hack_xy_from_ij(Ox_ref, Oy_ref, i_ref,  j_ref, x, y, dxdi, dydj, x_sys, y_sys);
    ROS_INFO("x_sys,y_sys = %f, %f",x_sys,y_sys);
    //compute arrival time based on distance from previous point to new point
    if(first_point) { 
        arrival_time=0;
        first_point=false;
        ROS_INFO("first point: arrival time set to zero");
    }
    else {
        double dx,dy,move_dist,move_time;
        dx = x_sys-x_sys_prev;
        dy = y_sys-y_sys_prev;
        move_dist = sqrt(dx*dx+dy*dy);
        move_time = move_dist/paint_speed;
        ROS_INFO("computed move dist = %f; move time = %f",move_dist,move_time);
        arrival_time+=move_time;
        }        
        
        
            //bool hack_xy_from_ij(float Ox_ref, float Oy_ref, int i_ref, int j_ref, int i_val, int j_val, float dxdi, float dydj, float &x_des, float &y_des) {
         paintfile<<x_sys<<", "<<y_sys<<", "<<sprayhead_vertical_offset<<", 0, 1, 0,    0, 0, -1, "<<sprayhead_on<<", " <<arrival_time<<endl;
         x_sys_prev= x_sys;  //save values for next point
         y_sys_prev= y_sys;
    }
    else if  ( event == EVENT_RBUTTONDOWN )
    {
        int npts = vertices.size();
        if (npts<1) { 
            ROS_WARN("cannot terminate polyline; need at least 2 vertices");
        }
        else {
                 ROS_INFO("terminating polyline with sprayhead off");
                cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
                vertex.x = x;
                vertex.y = y;
                vertices.push_back(vertex);
                //end of polyline; turn sprayhead off
                sprayhead_on=false; 
                spray_on_vec.push_back(sprayhead_on);
                hack_xy_from_ij(Ox_ref, Oy_ref, i_ref,  j_ref, x, y, dxdi, dydj, x_sys, y_sys);
                ROS_INFO("x_sys,y_sys = %f, %f",x_sys,y_sys);
                double dx,dy,move_dist,move_time;
                dx = x_sys-x_sys_prev;
                dy = y_sys-y_sys_prev;
                move_dist = sqrt(dx*dx+dy*dy);
                move_time = move_dist/paint_speed;
                ROS_INFO("computed move dist = %f; move time = %f",move_dist,move_time);
                arrival_time+=move_time;
                arrival_time+=move_time;               
                paintfile<<x_sys<<", "<<y_sys<<", "<<sprayhead_vertical_offset<<", 0, 1, 0,    0, 0, -1, "<<sprayhead_on<<", " <<arrival_time<<endl;                       
                x_sys_prev= x_sys;  //save values for next point
                y_sys_prev= y_sys;
        }


    }
    else if  ( event == EVENT_MBUTTONDOWN )
    {
        //cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        ROS_INFO("middle button clicked; closing the paintfile. ");
        paintfile.close();

    }
    /*
    else if ( event == EVENT_MOUSEMOVE )
    {
        cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
    }    */
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
         //set the callback function for any mouse event
     //setMouseCallback(OPENCV_WINDOW, ImageConverterMouseCB, NULL);
     setMouseCallback(OPENCV_WINDOW, ImageConverterMouseCB, NULL);
     //    minimal_subscriber_ = nh_.subscribe("example_class_input_topic", 1, &ExampleRosClass::subscriberCallback,this);  

  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
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
    Mat src = cv_ptr->image;
    resize(src, src_smaller, Size(src.cols/rescale_factor,src.rows/rescale_factor));
    imshow(OPENCV_WINDOW,src_smaller);    
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
  ros::init(argc, argv, "draw_polylines");
  ImageConverter ic;
  ros::NodeHandle nh;
  ros::Publisher pub_point =
    nh.advertise<geometry_msgs::Point>("point_wrt_sys_ref_frame", 1, true);

    // make this publisher available to callback
    g_pub_point = pub_point;
  g_affine_cam_wrt_sys = get_hardcoded_affine_cam_wrt_sys();
  g_nom_surface_normal<<0,0,1; //default ideal plane
  //paintfile.open("interactive_paintfile.cpf",ios::out|ios::trunc);
  vertices.clear();
  ros::spin();
  return 0;
}
