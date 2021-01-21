//rbip_image_projector.cpp
//this node is part of the image pipeline
// it takes in an image that is already rectified and transformed into an ideal, top-down-view virtual image
// however, there is still a zoom effect from variations in elevation of the surface being viewed
// This node requests a surface map from the mapper node (from LIDAR) as z[ix][jy]
// It transforms the input image to compute a synthetic image corresponding to projection onto the LIDAR (ideal) plane
// The intent is for images of symbols to fit the size of fixed templates
// Painting will still be performed to computed heights (per the height map), but user graphical overlays will
// use this projected image to get correct scaling and correct (x,y) values for painting

// correction for surface-map height could be very slow
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
//#include <camera_parameters/stella_amcrest_ipcam_params.h>
//#include <camera_parameters/CamCalib.h>

#include<roadprintz_msgs/VecOfDoubles.h>

int g_image_xc = g_virt_image_width/2;
int g_image_yc = g_virt_image_height/2;
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


static const std::string OPENCV_WINDOW = "ImageWindow";

Eigen::Affine3d g_affine_virt_cam_wrt_cam;

//global pointer for surface map service:
ros::ServiceClient *g_surface_map_service_client_ptr;
pcl_generator::MapperSrv g_map_srv;

std::vector<std::vector<double>> g_filtered_surface_map;



cv::Mat g_virtual_image; //(Nu,Nv,CV_8U,cv::Scalar(0));
cv::Mat g_src;  // need access to this; subsribe?
sensor_msgs::ImagePtr g_image_msg_ptr;
int g_N_X; //= srv.response.map_nx;
int g_N_Y; // = srv.response.map_ny;
double g_Y_RES; // = srv.response.map_y_res;
double g_X_RES; // = srv.response.map_x_res;
double g_X_MIN; // = srv.response.map_x_min;
double g_Y_MIN; // = srv.response.map_y_min;

//precompute how u_virt,v_virt map ont u_cam,v_cam
//hack: assign black to corner (0,0) in physical image, and
// for all (u_virt,v_virt) coordinates that do not have a correspondence in physical image,
// assign u_cam,v_cam = 0,0
int u_cam_mappings[g_virt_image_width][g_virt_image_height];
int v_cam_mappings[g_virt_image_width][g_virt_image_height];


bool g_got_new_image = false;
bool g_first_time = true;
bool g_got_transforms = false;
bool g_got_new_map = false;

bool g_verbose_interp = false;

int g_ans;

//issue here: topics to subscribe and publish
//HMI expects "/camera/image_rect_color" (actually, compressed version)
//but want to pipe this through this perspective transform first
std::string g_output_image_topic="/camera/image_rect_color";
std::string g_input_image_topic="/virtual_camera/image_rect_color";




//define an ideal transform for the virtual camera;
//may choose origin to be close to physical camera origin, so field of view is comparable
//this fnc should be OK, since it gets its params from header file
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

//convert (x,y) metric to indices of height-map table
//these are NOT the same as pixels!
int y_to_j(double y) {
    int j = 0;
    j = floor((y - g_Y_MIN) / g_Y_RES);
    if (j < g_N_Y && j>-1) return j;
    else return -1;
}

int x_to_i(double x) {
    int i = 0;

    i = floor((x - g_X_MIN) / g_X_RES);
    if (i < g_N_X && i>-1) return i;
    else return -1;
}

int y_to_j_round(double y) {
    int j = 0;
    j = round((y - g_Y_MIN) / g_Y_RES);
    if (j < g_N_Y && j>-1) return j;
    else return -1;
}

int x_to_i_round(double x) {
    int i = 0;

    i = round((x - g_X_MIN) / g_X_RES);
    if (i < g_N_X && i>-1) return i;
    else return -1;
}

//convert pixels to meters, based on virtual image params
//image x corresponds to RBIP y
double ux_to_RBIP_y(int ux) {
    double y = 0.0;
    y = ((double) (ux-g_image_xc))/KPIX; //*g_Y_RES; //+Y_RES/2;
        if (g_verbose_interp) {
        ROS_INFO("ux = %d; y= %f;  KPIX= %f",ux,y,KPIX);               
    }
    return y;
}

//image y corresponds to RBIP x
double vy_to_RBIP_x(int vy) {
    double x = VIRT_CAM_X_OFFSET + ((double) (vy-g_image_yc))/KPIX; //+X_RES/2;
    if (g_verbose_interp) {
        ROS_INFO("vy = %d; x= %f; VIRT_CAM_X_OFFSET = %f, KPIX= %f",vy,x,VIRT_CAM_X_OFFSET,KPIX);
                
    }
    return x;
}



bool interpolate_z_of_xy(double x, double y, double &z) {
    int i = x_to_i_round(x);
    int j = y_to_j_round(y);
    if (g_verbose_interp)  ROS_INFO("x_to_i, y_to_j = %d, %d",i,j);
    if ((i < 0) || (j < 0) || (i> g_N_X-2) || (j> g_N_Y-2) ) {
        z=0; //default if cannot interpolate
        return false; 
    }//impossibly high z-value if ask for values out of range
    //if (i >=g_N_X - 1) return false;
    //if (j >= g_N_Y - 1) return false;
    //if here, i and j can be interpolated to i+1, j+1
    //g_unfiltered_surface_map can be upgraded to filtered: TODO
    double z00 = g_filtered_surface_map[i][j];
    double z10 = g_filtered_surface_map[i + 1][j];
    double z01 = g_filtered_surface_map[i][j + 1];
    double z11 = g_filtered_surface_map[i + 1][j + 1];
    double dx = (x - (g_X_MIN + i * g_X_RES)) / g_X_RES;
    double dy = (y - (g_Y_MIN + j * g_Y_RES)) / g_Y_RES;
    //unit-square formula: z(x,y) = z00*(1-dx)*(1-dy) + z10*dx*(1-dy)+z01*(1-dx)*dy+z11*dx*dy
    z = z00 * (1 - dx)*(1 - dy) + z10 * dx * (1 - dy) + z01 * (1 - dx) * dy + z11 * dx*dy;

    if (g_verbose_interp)ROS_INFO("interpolator: x,y = %f, %f; i,j= %d, %d",x,y,i,j);
    if (g_verbose_interp)ROS_INFO("z00, z10, z01, z11 = %f, %f, %f, %f",z00,z10,z01,z11);
    if (g_verbose_interp)ROS_INFO("dx, dy = %f, %f; interpolated z = %f",dx,dy,z);
    //return z;
    return true;
}


bool valid_uv(int u,int v,int Nu,int Nv) {
  if (u<0) return false;
  if (v<0) return false;
  if (u>Nu-1) return false;
  if (v>Nv-1) return false;
  return true;
}

//pre-compute the mappings between (u,v) of virtual camera and (u,v) of physical camera
//use this approach:

//step through all u_virt, v_virt pixel indicies for projected image to be populated
//compute u_cam, v_cam of input image to find RGB values to use for projected image at u_virt,v_virt
//use these relations:
// u_virt,v_virt--> px,py in metric;
// pz = z_map(px,py)
// Ocam_z = height of virtual camera for input image
//proj_shift_factor = Ocam_z/(Ocam_z-pz); 
// u_cam = proj_shift_factor*(u_virt - u_virt_xc) + u_virt_xc
// v_cam = proj_shift_factor*(v_virt - v_virt_xc) + v_virt_xc
// pz is height of LIDAR map at (x,y)
void compute_mappings() //Eigen::Affine3d affine_virt_cam_wrt_cam, 
  //double cx_src,double cy_src, double fx_src, double fy_src, double vert_height, double kpix)
{
    //int u_cam_mappings[g_rect_width][g_rect_height];
    //int v_cam_mappings[g_rect_width][g_rect_height];
    double cx_virt = g_virt_image_width/2.0;
    double cy_virt = g_virt_image_height/2.0;
    Eigen::Vector3d p_wrt_virt,p_wrt_cam;
  int u_cam,v_cam;
  double x,y,z;
  double proj_shift_factor;// = Ocam_z/(Ocam_z-pz);
  //p_wrt_virt[2] = vert_height;
  //reprojected image has same dimensions as virtual image
 for (int u_virt=0;u_virt<g_virt_image_width;u_virt++) {
  for (int v_virt=0;v_virt<g_virt_image_height;v_virt++) {
      u_cam_mappings[u_virt][v_virt] = 0; //default
      v_cam_mappings[u_virt][v_virt] = 0;

      //THIS IS WHERE THE HARD WORK GOES...
      /*
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
    */
      y = ux_to_RBIP_y(u_virt);
      x = vy_to_RBIP_x(v_virt);
      
    //  ROS_INFO("x,y = %f, %f",x,y);
    interpolate_z_of_xy(x,y,z);  //z should contain height of pt at (x,y)
    proj_shift_factor = VERT_CAM_HEIGHT/(VERT_CAM_HEIGHT-z);
    u_cam = round(proj_shift_factor*(u_virt - cx_virt) + cx_virt);
    v_cam = round(proj_shift_factor*(v_virt - cy_virt) + cy_virt);  
    
    //debug: display focus regions:
    /*
    if ((u_virt>2800)&&(u_virt<2810)&&(v_virt>1800)&&(v_virt<1810)) {
        g_verbose_interp=true;
        y = ux_to_RBIP_y(u_virt); //y, in RBIP coords
        x = vy_to_RBIP_x(v_virt); //x, in RBIP coords
        interpolate_z_of_xy(x,y,z);
        g_verbose_interp=false;
        ROS_INFO("x,y,z, proj_shift_factor = %f, %f, %f,%f",x,y,z, proj_shift_factor);
        ROS_INFO("u_virt, u_cam, v_virt, v_cam = %d, %d, %d, %d",u_virt, u_cam, v_virt, v_cam);        
    }*/
    /*
    if (z>0.1) {
    ROS_INFO("z, proj_shift_factor = %f,%f",z, proj_shift_factor);
    ROS_INFO("u_virt, u_cam, v_virt, v_cam = %d, %d, %d, %d",u_virt, u_cam, v_virt, v_cam);
            
    }*/
    
    //validity w/rt dimensions of pre-transformed image:
    if (valid_uv(u_cam,v_cam,g_virt_image_width,g_virt_image_height)) { 
        u_cam_mappings[u_virt][v_virt] = u_cam;
        v_cam_mappings[u_virt][v_virt] = v_cam;
    } 
    else {
     //   ROS_WARN("map out of range: u_cam, v_cam, u_virt, v_virt, height,width: %d %d %d, %d, %d, %d",u_cam,v_cam,u_virt,v_virt,g_virt_image_height,g_virt_image_width);
    }
  }
 }
  ROS_INFO("compute_mappings concluded");
    
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

    //cv::namedWindow(OPENCV_WINDOW);
    //cv::namedWindow(OPENCV_ZOOM_WINDOW);
         //set the callback function for any mouse event
     //setMouseCallback(OPENCV_WINDOW, ImageConverterMouseCB, NULL);
     //setMouseCallback(OPENCV_WINDOW, ImageConverterMouseCB, NULL);
     //setMouseCallback(OPENCV_ZOOM_WINDOW, ImageConverterZoomedMouseCB, NULL);
     //    minimal_subscriber_ = nh_.subscribe("example_class_input_topic", 1, &ExampleRosClass::subscriberCallback,this);  

  }

  ~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
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

    /*if (!g_got_transforms) {
        ROS_WARN("got image, but transforms not ready yet");
        return;
    }*/
    
    //ask the surface mapper for the array z[ix][jy]:
        //
        ROS_INFO("Requesting map matrix");
        
        g_surface_map_service_client_ptr->call(g_map_srv);
        bool response_code= g_map_srv.response.response_code;
        ROS_INFO("service returned %d",response_code);
        if (response_code!=pcl_generator::MapperSrvResponse::SUCCESS) {
            
            ROS_ERROR("failed response from mapper service");
            g_got_new_image=false;
            g_got_new_map=false;
            return;
        }
        else { //got a  map; unpack it
            if (g_first_time) {
                g_N_X = g_map_srv.response.map_nx;
                g_N_Y = g_map_srv.response.map_ny;
                g_Y_RES = g_map_srv.response.map_y_res;
                g_X_RES = g_map_srv.response.map_x_res;
                g_X_MIN = g_map_srv.response.map_x_min;
                g_Y_MIN = g_map_srv.response.map_y_min;

                ROS_INFO("N_X, N_Y = %d, %d",g_N_X,g_N_Y);
                ROS_INFO("X_MIN,Y_MIN = %f, %f",g_X_MIN,g_Y_MIN);
                ROS_INFO("X_RES,Y_RES = %f, %f",g_X_RES,g_Y_RES);
                //cout<<"enter 1: ";
                //cin>>g_ans;
            }
        
            //convert the received vec of vecs message into a std vec of vecs 
            int nvecs = g_map_srv.response.vec_of_vec_of_doubles.size();//vec_of_vec_of_doubles
            ROS_INFO("vec of vecs contains %d vectors",nvecs);
            ROS_INFO("unpacking the vec of vecs...");
            std::vector<double> vec_of_dbls;
            g_filtered_surface_map.clear();
            roadprintz_msgs::VecOfDoubles vec_of_dbls_msg;
            for (int ivec=0;ivec<nvecs;ivec++) {
                vec_of_dbls_msg = g_map_srv.response.vec_of_vec_of_doubles[ivec];//vec_of_vec_of_doubles
                vec_of_dbls=vec_of_dbls_msg.dbl_vec;
                g_filtered_surface_map.push_back(vec_of_dbls);
            }
            ROS_INFO("received/repackaged map vec of vec of doubles: ");
            g_got_new_map=true;
        }
    
    if (g_first_time) {
      cout<<"first image received; cloning to transformed image"<<endl;
      g_virtual_image = g_src.clone();
      //cv::resize(img, img_dst, cv::Size(640, 480), 0, 0, cv::INTER_AREA);
      //resize(g_src,g_virtual_image,Size(g_virt_image_width,g_virt_image_height),0,0,cv::INTER_AREA);
      g_first_time=false;
      if (g_src.cols != g_virt_image_width) {
          ROS_ERROR("image is not the expected size; exiting");
          exit(0);
      }
      if (g_src.rows != g_virt_image_height) {
          ROS_ERROR("image is not the expected size; exiting");
          exit(0);
      }      
     cout<<"input image size height, width: "<<g_src.cols<<", "<<g_src.rows<<endl;
     cout<<"setting all pixels of RBIP-projected image to black"<<endl;
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
     //imshow(OPENCV_WINDOW, g_virtual_image);
     //cout<<"enter 1 to continue: ";
     //cin>>g_ans;
    //cv::waitKey(3);

    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
    g_image_msg_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", g_virtual_image).toImageMsg();
    //image_pub_.publish(g_virtual_image);
    image_pub_.publish(g_image_msg_ptr);

   
    
      return;
    }
        
    //ROS_INFO("computing reprojection pixel mappings...");

    //ugh.  maybe the virtual and real camera parameters headers should live in their own package!
    
    compute_mappings(); //g_affine_virt_cam_wrt_cam,g_image_xc,g_image_yc); //, g_fx, g_fy, VERT_CAM_HEIGHT,KPIX);

    //ROS_INFO("done computing mappings"); 
    
    //ROS_INFO("transforming image...");
    transform_image(g_src,g_virtual_image);
    //cout<<"done transforming image"<<endl;
    //resize(g_virtual_image, dst_smaller, Size(g_virtual_image.cols/RESCALE_FACTOR,g_virtual_image.rows/RESCALE_FACTOR));
    //imshow(OPENCV_WINDOW, dst_smaller);  

    //cv::waitKey(3);

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
  ros::init(argc, argv, "rbip_image_projector");
  ImageConverter ic;



 Eigen::Affine3d affine_cam_wrt_RBIP,affine_virt_cam_wrt_RBIP;
 // ImageConverter ic; //reuse imageConverter from interactive_image_gui2.cpp
 // this will receive images; use it to populate g_cam_image
 // have this fnc set g_got_new_image
  ros::NodeHandle nh;
  /* these transforms are not needed for this RBIP projection node
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
   */
  
 //affine_cam_wrt_RBIP = get_hardcoded_affine_cam_wrt_sys();
 affine_virt_cam_wrt_RBIP = get_hardcoded_affine_virtual_cam_wrt_RBIP();
 g_affine_virt_cam_wrt_cam = affine_cam_wrt_RBIP.inverse()*affine_virt_cam_wrt_RBIP;
  g_got_transforms=true;

  //the only thing this does is informs the HMI of the value of KPIX, which is in the virtual-camera header file
  //ros::ServiceServer calib_service = nh.advertiseService("cam_calib_data_service", cam_calib_callback);

      
      
  ros::ServiceClient client = nh.serviceClient<pcl_generator::MapperSrv>("surface_map_service");
  g_surface_map_service_client_ptr = &client;
  g_map_srv.request.request_code=pcl_generator::MapperSrvRequest::REQUEST_MAP_MATRIX;
 
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
   ros::Duration(0.1).sleep();

  }
 
}


