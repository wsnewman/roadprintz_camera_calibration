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
//#include <xform_utils/xform_utils.h>

static const std::string OPENCV_WINDOW = "ImageWindow";
using namespace cv;
using namespace std;

std::vector<cv::Point> vertices;
cv::Point vertex;
ofstream paintfile;
double arrival_time=0;

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
        vertex.x = x;
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
    int n_vertices = vertices.size();
    if (n_vertices>1) {
        for (int i=0;i<n_vertices-1;i++) {
            if(spray_on_vec[i]) {
                line(cv_ptr->image,vertices[i],vertices[i+1],cv::Scalar( 255, 0, 0 ),8);
            }
        }
    }
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
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
  paintfile.open("interactive_paintfile.cpf",ios::out|ios::trunc);
  vertices.clear();
  ros::spin();
  return 0;
}