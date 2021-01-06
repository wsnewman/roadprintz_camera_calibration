// pgm specialized to analyze image16_rect.png, which seems close to desired pose, elbow-angle = 0.25



//test program for camera calib
// process: obtain corresponding (x,y,z)/sys -> (i,j) points
// solve first for linear intrinsic and extrinsic params
// use these values to find nonlinear coeff's
// use this data in ipcam_driver to publish dewarped images
// after dewarping, obtain new (x,y,z)/sys -> (i,j) points from dewarped image
// compute corresponding linear intrinsic and extrinsic params
// use these alternative parameters to get (x,y)/sys from (i,j)/image

//lots of dead code copied from Ingenuity paint gui



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
using namespace Eigen;

/// Global variables
int g_click_num=0;
int g_x0,g_y0,g_x1,g_y1;

Mat src, src_gray;
Mat dst, dst2, detected_edges;
int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
static const std::string window_name = "Edge Map";
static const std::string analysis_window_name = "analysis";
//char* window_name = "Edge Map";

vector <double> rms_errs, slopes, offsets;
vector <int> n_edge_pixels_vec;
int g_ans;


std::vector<cv::Point> vertices;
std::vector<cv::Point> corner_vertices;
cv::Point vertex;
ofstream paintfile;


Eigen::Matrix3d R_sys_wrt_cam, R_cam_wrt_sys;
Eigen::Matrix3d K_camera, K_camera_inv;
Eigen::Vector3d O_sys_wrt_cam, O_cam_wrt_sys;
Eigen::Affine3d T_sys_wrt_cam, T_cam_wrt_sys;
Eigen::Vector3d uv_vec, p_wrt_cam, p_wrt_sys;
Eigen::VectorXd x_vec, y_vec;
//MAGIC NUMBERS: should get these from camera parameters pub
//intrinsic params:
double fx = 471.35; //469.14;
double fy = 429.97; //423.30;
//define central pixel offset; 
double i_c = 385.01; //371.1;// %352;
double j_c = 225.06; //220.0; //%240;
//est_O_w_wrt_c =  -0.15000  -2.37000   2.91200
//extrinsic params
double Ox_w_wrt_c = -0.150; //0.0600;
double Oy_w_wrt_c = -2.370; //-2.604;
double Oz_w_wrt_c = 2.932; //-2.932;
//psi_identified =  1.6100
double phi_z = 1.61; //1.610; //1.53;
double theta_x = 3.14159;

//   Rotx = [1,0,0;
//       0,cos(theta_x),-sin(theta_x);
//       0,sin(theta_x),cos(theta_x)]
//       
//   Rotz = [cos(phi_z),-sin(phi_z),0;
//       sin(phi_z),cos(phi_z),0;
//       0,0,1]
//   R_sys_wrt_cam=Rotz*Rotx      

//   O_w_wrt_c= [0.060000,  -2.604000,  -2.932000]
//phi_z =  1.53%1.5400;
//Rotz = [cos(phi_z),-sin(phi_z),0;
//        sin(phi_z),cos(phi_z),0;
//        0,0,1];
//R_w_wrt_cam = Rotz; % assumes tilt about x and y are negligible

/**
 * @function CannyThreshold
 * @brief Trackbar callback - Canny thresholds input with a ratio 1:3
 */
void CannyThreshold(int, void*) {
    cv::Point vertex1, vertex2;
    /*
vertex.x = 447;  //
vertex.y = 74;   */
    //line(cv_ptr->image, corner_vertices[0], corner_vertices[1], cv::Scalar(255, 0, 0), 8);

    /// Reduce noise with a kernel 3x3
    blur(src_gray, detected_edges, Size(3, 3));

    /// Canny detector
   lowThreshold=max_lowThreshold; //wsn override slider
    cout<<"lowThreshold = "<<lowThreshold<<endl;
    //cout<<"enter 1: ";
    //        cin>>g_ans;
    Canny(detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size);

    //analyze detected_edges: ranges specialized for "nom_image.jpg"
    int n_edge_pixels = 0;
    double kx, jy;
    Scalar intensity; // = img.at<uchar>(y, x);
    rms_errs.clear();
    slopes.clear();
    offsets.clear();
    n_edge_pixels_vec.clear();
    vector <double> x_vec_dbl, y_vec_dbl; //,rms_errs,slopes,offsets;
    //((442,300), (588,516)
    for (int i = 440; i < 590; i++) {
        n_edge_pixels = 0;
        x_vec_dbl.clear();
        y_vec_dbl.clear();
        for (int k = i - 2; k <= i + 2; k++) {
            int kpixels = 0;
            for (int j = 300; j < 520; j++) {


                //cv_ptr->image.at<cv::Vec3b>(j, i)[0] = 255;
                intensity = detected_edges.at<uchar>(j, k);
                //cout<<"intensity: "<<intensity<<endl;
                if (intensity.val[0] > 100) {
                    n_edge_pixels++;
                    kpixels++;
                    kx = (double) k;
                    jy = (double) j;
                    x_vec_dbl.push_back(kx);
                    y_vec_dbl.push_back(jy);
                    //cout<<"kx, jy = "<<kx<<", "<<jy<<endl;
                }
            }
            cout << "k, kpixels = " << k << ", " << kpixels << endl;
            /*
            if (kpixels>100) {
                cout<<"enter 1 to continue: ";
                cin>>g_ans;
            }*/
        }
        cout << "i = " << i << "; n_edge_pixels in vertical band = " << n_edge_pixels << endl;
        int npts = x_vec_dbl.size();
        cout << "saved " << npts << " values in x,y vectors" << endl;
        x_vec.resize(npts);
        y_vec.resize(npts);
        Eigen::MatrixXd Amat;
        Amat = Eigen::MatrixXd::Ones(npts, 2);

        /**/
        for (int m=0;m<npts;m++) {
            
                Amat(m,0) = y_vec_dbl[m];
                x_vec(m) = x_vec_dbl[m];
                if (i==344) {
                    cout<<"m, x, Amat(m,0), Amat(m,1)= "<<m<<", "<<x_vec(m)<<", "<<Amat(m,0)<<","<<Amat(m,1)<<endl;
                }
            }
         /**/
        //solve x = m*y+b, or x_vec = [y_vec,ones][m;c]

        Eigen::Vector2d slope_offset;
        //Eigen::VectorXd err_vec;
        //err_vec = Amat.bdcSvd(ComputeThinU | ComputeThinV).solve(y_vec);
        slope_offset = Amat.bdcSvd(ComputeThinU | ComputeThinV).solve(x_vec);
        cout << "The least-squares solution is:"
                << slope_offset.transpose() << endl; // Amat.bdcSvd(ComputeThinU | ComputeThinV).solve(y_vec) << endl;
        /*
        if (fabs(err_vec[0] > 1)) {
            cout << "unusual result; input data: " << endl;
            for (int i = 0; i < npts; i++) {
                cout << "x,y = " << x_vec_dbl[i] << ", " << y_vec_dbl[i] << endl;
            }


        }*/

        //slope_offset = Amat.inverse()*x_vec;
        //compute rms error:
        Eigen::VectorXd err_fit_vec;
        err_fit_vec = x_vec - Amat*slope_offset;

        double sum_sqrd = err_fit_vec.transpose() * err_fit_vec;
        double rms = sqrt(sum_sqrd / npts);
        rms_errs.push_back(rms);
        n_edge_pixels_vec.push_back(n_edge_pixels);

        cout << "rms_err = " << rms << endl;
        double slope, offset;

        slope = slope_offset(0);
        offset = slope_offset(1);
        cout << "slope, offset = " << slope << ", " << offset << endl;
        slopes.push_back(slope);
        offsets.push_back(offset);

        if (n_edge_pixels > 300) {
            //cout<<"enter 1: ";
            //cin>>g_ans;
        }

    }
    cout << "XXXXXXXXXXXXXXXXX   summary  XXXXXXXXXXXXXXXXXXX" << endl;
    int ncols = slopes.size();
    dst2 = src.clone();
    double slope, offset, rms;
    cout << "size of slopes = " << ncols << endl;
    for (int i = 0; i < ncols; i++) {
        rms = rms_errs[i];
        slope = slopes[i];
        offset = offsets[i];
        int colnum = i + 280;
        cout << "colnum, rms err, slope, offset = " << colnum << ", " << rms << ", " << slope << ", " << offset << endl;


        //for (int j = 190; j < 370; j++)
        int band_pixels = 0;
        for (int k = colnum - 4; k <= colnum + 4; k++) {
            double npixels = 0;
            for (int j = 190; j < 370; j++) {
                intensity = detected_edges.at<uchar>(j, k);
                //cout<<"intensity: "<<intensity<<endl;
                if (intensity.val[0] > 100) {
                    //cout<<" k, j = "<<k<<", "<<j<<endl;
                    npixels++;
                    band_pixels++;
                }
            }
            //cout<<"column "<<k<<" npixels = "<<npixels<<endl;
        }
        cout << "band_pixels = " << band_pixels << endl;
        if (band_pixels > 200 && rms < 1.5) {
            int npixels = 0;
            vertex1.y = 190;
            vertex2.y = 370;
            vertex1.x = (int) (slope * 190 + offset);
            vertex2.x = (int) (slope * 370 + offset);
            cout << "attempting to draw line from x1,y1 = " << vertex1.x << "," << vertex1.y << " to x2,y2 = " << vertex2.x << "," << vertex2.y << endl;
            //line(dst2, vertex1, vertex2, cv::Scalar(128, 0, 0), 2);
            //cout<<"line derived from the following data: "<<endl;
        }


    }

    /// Using Canny's output as a mask, we display our result
    dst = Scalar::all(0);

    src.copyTo(dst, detected_edges);
    imshow(window_name, dst);

    //make a grid template, after corners have been specified:
    if (g_click_num==2) {
        g_x0 = 458; //approx soln f
        g_y0 = 306;
        g_x1 = 570;
        g_y1 = 498;
        
        /*
Left button of the mouse is clicked - position (307, 242)
upper left corner set to 307, 242
Left button of the mouse is clicked - position (421, 433)
lower-right corner set to 421, 433
         */
        int x_start = g_x0;
        int x_end = g_x1;
        vertex1.y = g_y0;
        vertex2.y = g_y1;
        double dx = (x_end-x_start)/7.0;
        for (int i=0;i<8;i++) {
            vertex1.x = round(x_start+i*dx);
            vertex2.x = vertex1.x;
            line(dst2, vertex1, vertex2, cv::Scalar(0, 0, 128), 1);
        }
        int y_start = g_y0;
        int y_end = g_y1;
        vertex1.x = g_x0;
        vertex2.x = g_x1;
        double dy = (y_end-y_start)/12.0;
        for (int i=0;i<14;i++) {
            vertex1.y = round(y_start+i*dy);
            vertex2.y = vertex1.y;
            line(dst2, vertex1, vertex2, cv::Scalar(0, 0, 128), 1);
        }        
        //
        vertex1.x = x_start+3*dx;
        vertex1.y = y_start+dy;
        cout<<"near laser ref: "<<vertex1.x<<", "<<vertex1.y<<endl;
        cv::circle(dst2, vertex1, 3, CV_RGB(255, 0, 0), 2);
        vertex1.y = y_start+13*dy;
        cv::circle(dst2, vertex1, 3, CV_RGB(255, 0, 0), 2);
        imwrite( "./image16_markup.jpg", dst2 );
        cout<<"far laser ref: "<<vertex1.x<<", "<<vertex1.y<<endl;

    }
        imshow(analysis_window_name, dst2);
    cout<<"lowThreshold = "<<lowThreshold<<endl;
      
    //}

}

bool xy_from_ij(float Ox_ref, float Oy_ref, int i_ref, int j_ref, int i_val, int j_val, float dxdj, float dydi, float &x_des, float &y_des) {
    /*
      Eigen::Matrix2f scale_mat;
      Eigen::Vector2f col0, col1;
      col0 << dxdj / 1000.0, 0.0; //convert to m/pixel
      col1 << 0.0, dydi / 1000.0;
      scale_mat.col(0) = col0;
      scale_mat.col(1) = col1;
      //scale_mat[0][0] = dxdi;
      //scale_mat[0][1] = 0.0;
      //scale_mat[1][0] = 0.0;
      //scale_mat[1][1] = dydj;
      Eigen::Vector2f ij_ref, ij_des, xy_ref, xy_des;
      ij_des << i_val, j_val;
      ij_ref << i_ref, j_ref;
      xy_ref << Ox_ref, Oy_ref;
      //xy_des << x_des, y_des;
      xy_des = scale_mat * (ij_des - ij_ref) + xy_ref;
    
      ROS_INFO_STREAM("ij_ref = " << ij_ref.transpose() << endl);
      ROS_INFO_STREAM("ij_des = " << ij_des.transpose() << endl);
      ROS_INFO_STREAM("xy_ref = " << xy_ref.transpose() << endl);
      ROS_INFO_STREAM("xy_des = " << xy_des.transpose() << endl);

      x_des = xy_des[0];
      y_des = xy_des[1];
     * */
    //x_robot = 2.75 + dx/dj*(jpix - 521)
    dxdj /= 1000.0; //convert to m/pix
    dydi /= 1000.0; // ditto
    x_des = Ox_ref + dxdj * (j_val - j_ref);
    //y_robot = 0 + dy/di* (ipix - 944) 
    y_des = Oy_ref + dydi * (i_val - i_ref);
    //inv(scale_mat)*(xy_des-xy_ref)+ij_ref = ij_des;  test if result is valid; return values and bool
    //ij_des = scale_mat.inverse()*(xy_des-xy_ref) + ij_ref;
    //i_val = ij_des[0];
    //j_val = ij_des[1];
    if ((i_val >= 448)&& (i_val < 1436) &&(j_val >= 10)&&(j_val < 1000)) {
        return true;
    } else {
        return false;
    }

}

//generic code to add a new vertex of a polyline--not the first move-to point:

void add_line(double x_sys, double y_sys, double x_sys_prev, double y_sys_prevn) {
    double dx, dy, move_dist, move_time;
    /*
              dx = x_sys - x_sys_prev;
              dy = y_sys - y_sys_prev;
              move_dist = sqrt(dx * dx + dy * dy);
              if (move_dist < 0.001) move_dist = 0.001;
              dx /= move_dist;
              dy /= move_dist;
              //use this tangent as goal for move-to; 
              move_time = move_dist / PAINT_SPEED;  
              arrival_time += move_time;
              paintfile << x_sys << ", " << y_sys << ", " << SPRAYHEAD_VERTICAL_OFFSET << ", " << dx << ", " << dy << ", 0,  0, 0, -1,  1, " << arrival_time << endl;    

     */
}

void ImageConverterMouseCB(int event, int x, int y, int flags, void* userdata) {

    if (event == EVENT_LBUTTONDOWN) {
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        if (g_click_num==0) {
            g_x0=x;
            g_y0=y;
            cout<<"upper left corner set to "<<g_x0<<", "<<g_y0<<endl;
            g_click_num++;
        }
        else if (g_click_num==1) {
            g_x1=x;
            g_y1=y;
            cout<<"lower-right corner set to "<<g_x1<<", "<<g_y1<<endl;
            g_click_num++;            
        }
        /*
        //compute coords in sys frame from pixels using:
        //[x',y',1]' = K_camera_inv*[i,j,1]'
        //p_wrt_cam = [x',y',1]'*O_sys_wrt_cam(2)
        //p_wrt_sys = R_cam_wrt_sys*p_wrt_cam + O_cam_wrt_sys
        uv_vec << x, y, 1;
        cout << "uv_vec: " << uv_vec.transpose() << endl;
        Eigen::Vector3d p_normalized, p_wrt_cam;
        p_normalized = K_camera_inv*uv_vec;
        cout << "p_normalized: " << p_normalized.transpose() << endl;
        p_wrt_cam = O_sys_wrt_cam(2) * p_normalized;
        cout << "p_wrt_cam: " << p_wrt_cam.transpose() << endl;

        p_wrt_sys = R_cam_wrt_sys * (p_wrt_cam - O_sys_wrt_cam);
        cout << "p_wrt_sys = " << p_wrt_sys.transpose() << endl;
        */
    }

}

class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    //void ImageConverterMouseCB(int event, int x, int y, int flags, void* userdata);

public:

    ImageConverter()
    : it_(nh_) {
        // Subscribe to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/image_rect_color", 1,
                &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);

        cv::namedWindow(OPENCV_WINDOW);
        //set the callback function for any mouse event
        //setMouseCallback(OPENCV_WINDOW, ImageConverterMouseCB, NULL);
        setMouseCallback(OPENCV_WINDOW, ImageConverterMouseCB, NULL);
        //    minimal_subscriber_ = nh_.subscribe("example_class_input_topic", 1, &ExampleRosClass::subscriberCallback,this);  


        /// Create a window
        //namedWindow( window_name, CV_WINDOW_AUTOSIZE );

        /// Create a Trackbar for user to enter threshold
        //createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );



    }

    ~ImageConverter() {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }


        // Draw an example circle on the video stream
        //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        //  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

        int n_vertices = vertices.size();
        for (int ivertex = 0; ivertex < n_vertices; ivertex++) {
            cv::circle(cv_ptr->image, vertices[ivertex], 5, CV_RGB(255, 0, 0), 5);
        }
        /*
        int n_sprayhead_vals = spray_on_vec.size();
        //cout<<"imageCb: n_vertices = "<<n_vertices<<endl;
                
        if (n_vertices > 1) {
            for (int i = 0; i < n_sprayhead_vals - 1; i++) {
                //cout<<"vertex "<<i<<endl;
                //cout<<"spray_on[i+1] = "<<spray_on_vec[i+1]<<endl;
                if (spray_on_vec[i+1]) {
                    //cout<<"display line from pt "<<i<<" to pt "<<i+1<<endl;
                    line(cv_ptr->image, vertices[i], vertices[i + 1], cv::Scalar(255, 0, 0), 45);
                }
            }
        }
        line(cv_ptr->image, corner_vertices[0], corner_vertices[1], cv::Scalar(255, 0, 0), 8);
        line(cv_ptr->image, corner_vertices[1], corner_vertices[2], cv::Scalar(255, 0, 0), 8);
        line(cv_ptr->image, corner_vertices[2], corner_vertices[3], cv::Scalar(255, 0, 0), 8);
        line(cv_ptr->image, corner_vertices[3], corner_vertices[0], cv::Scalar(255, 0, 0), 8);  
         */
        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(100); //arg in msec

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());

    }
    //void find_lines( cv_bridge::CvImagePtr cv_ptr) {
    /// Show the image
    //CannyThreshold(0, 0);


    //}
};

//void ImageConverterMouseCB(int event, int x, int y, int flags, void* userdata);

int orig_main(int argc, char** argv) {
    ros::init(argc, argv, "extrinsic_calib");
    ImageConverter ic;

    O_sys_wrt_cam << Ox_w_wrt_c, Oy_w_wrt_c, Oz_w_wrt_c;
    cout << "O_sys_wrt_cam: " << O_sys_wrt_cam.transpose() << endl;

    Eigen::Vector3d x_vec, y_vec, z_vec, Rxvec1, Rxvec2, Rxvec3, Rzvec1, Rzvec2, Rzvec3;
    Eigen::Matrix3d Rotz, Rotx;
    //x_vec<<fx, 0.0, ic; //fx,0,ic;  //
    //x_vec<<cos(phi_z),sin(phi_z),0;
    //yvec<<0,fy,jc;
    //zvec<<0,0,1;
    //x_vec<<fx, 0, ic;
    x_vec << fx, 0, 0;
    y_vec << 0, fy, 0;
    z_vec << i_c, j_c, 1;
    K_camera.col(0) = x_vec;
    K_camera.col(1) = y_vec;
    K_camera.col(2) = z_vec;
    K_camera_inv = K_camera.inverse();
    cout << "K_camera: " << endl << K_camera << endl;

    cout << "K_camera_inv: " << endl << K_camera_inv << endl;
    //Eigen::Matrix3d Rotz;
    //   Rotx = [1,0,0;
    //       0,cos(theta_x),-sin(theta_x);
    //       0,sin(theta_x),cos(theta_x)]
    //       
    //   Rotz = [cos(phi_z),-sin(phi_z),0;
    //       sin(phi_z),cos(phi_z),0;
    //       0,0,1]
    //   R_sys_wrt_cam=Rotz*Rotx 
    Rxvec1 << 1, 0, 0;
    Rxvec2 << 0, cos(theta_x), sin(theta_x);
    Rxvec3 << 0, -sin(theta_x), cos(theta_x);
    Rotx.col(0) = Rxvec1;
    Rotx.col(1) = Rxvec2;
    Rotx.col(2) = Rxvec3;

    Rzvec1 << cos(phi_z), sin(phi_z), 0;
    Rzvec2 << -sin(phi_z), cos(phi_z), 0;
    Rzvec3 << 0, 0, 1;
    Rotz.col(0) = Rzvec1;
    Rotz.col(1) = Rzvec2;
    Rotz.col(2) = Rzvec3;

    R_sys_wrt_cam = Rotz*Rotx;
    cout << "R_sys_wrt_cam: " << endl << R_sys_wrt_cam << endl;

    R_cam_wrt_sys = R_sys_wrt_cam.inverse();
    O_cam_wrt_sys = -R_cam_wrt_sys*O_sys_wrt_cam;
    cout << "O_cam_wrt_sys: " << O_cam_wrt_sys.transpose() << endl;

    //compute coords in sys frame from pixels using:
    //[x',y',1]' = K_camera_inv*[i,j,1]'
    //p_wrt_cam = [x',y',1]'*O_sys_wrt_cam(2)
    //p_wrt_sys = R_cam_wrt_sys*p_wrt_cam + O_cam_wrt_sys




    paintfile.open("interactive_paintfile.cpf", ios::out | ios::trunc);
    vertices.clear();
    corner_vertices.clear();
    //points at values (x,y)_robot = (1.85,-1), (1.85,1)         (447, 74)  (1453, 82)
    //                                (2.75,0)           =         (944, 539)
    //                           (3.65,-1), (3.65,1)         (457, 968) (1420, 978)
    /*
    vertex.x = 447;  //
    vertex.y = 74;   
    corner_vertices.push_back(vertex);
    vertex.x = 1453;  //
    vertex.y = 82;   
    corner_vertices.push_back(vertex);
    vertex.x = 1420;  //
    vertex.y = 978;   
    corner_vertices.push_back(vertex);
    vertex.x = 457;  //
    vertex.y = 968;   
    corner_vertices.push_back(vertex);    
     */
    ros::spin();
    return 0;
}

/** @function main */
int main(int argc, char** argv) {
        ros::init(argc, argv, "rp_find_template"); //name this node

    ros::NodeHandle nh; 
    /// Load an image
    //src = imread(argv[1]);
    //src = imread("image16_rescaled.jpg");
    cout<<"enter image filename: ";
    string fname;
    cin>>fname;
    //src = imread("image16_rect_resized.png");
    src = imread(fname.c_str());

    if (!src.data) {
        cout<<"COULD NOT READ FILE"<<endl;
        return -1;
    }
    cout<<"should get size 660 rows, 1000 cols"<<endl;
    cout<<"got image size: "<<src.rows<<", "<<src.cols<<endl;
    /// Create a matrix of the same type and size as src (for dst)
    dst.create(src.size(), src.type());
    dst2 = src.clone();
    //dst2.create(src.size(), src.type());
    /// Convert the image to grayscale
    cvtColor(src, src_gray, CV_BGR2GRAY);
    //src.copyTo(src, dst2);
    //dst2 = src;
    /// Create a window
    namedWindow(window_name, CV_WINDOW_AUTOSIZE);
    namedWindow(analysis_window_name, CV_WINDOW_AUTOSIZE);
    setMouseCallback(analysis_window_name, ImageConverterMouseCB, NULL);

    //

    /// Create a Trackbar for user to enter threshold
    createTrackbar("Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold);

    /// Show the image
    CannyThreshold(0, 0);
    cout<<"click upper-left corner of template, then lower right, then slide image slider"<<endl;
    /*
    cout<<"click upper-left corner of template: ";
    while (      g_click_num ==0) {
        ros::Duration(0.1).sleep();
    }
    cout<<"click lower-right corner of template: ";
    while (      g_click_num ==1) {
        ros::Duration(0.1).sleep();
    }
    */
    /// Wait until user exits the program by pressing a key
    waitKey(0);

    return 0;
}
