// wsn, 3/2020; ceres optimizer for intrinsic calibration using images from mill with known Cartesian displacements
// try version with numerical gradient and avoiding templates


#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "glog/logging.h"


#include <Eigen/Eigen> //for the Eigen library
#include <Eigen/Dense>
#include <Eigen/Geometry>

//the following includes a function that reads in calibration data from disk
//in a specific format: pixel_i, pixel_j, target_x, target_y, sled_dx, sled_dy, sled_dz
// arbitrarily set sled frame origin coincident with target upper-left circle origin of image_0_0_0.jpg
// mill axes are approximately aligned with target-frame axes, though a perturbation refinement is
// part of the optimization (3-DOF param)
// want to find 9 intrinsic params, 6 extrinsic params (transform target0 frame w/rt camera frame) and 3 extrinsic rotation params (sled-to-target rotation)
#include "calibration_file_reader.cpp"

using ceres::AutoDiffCostFunction;
using ceres::NumericDiffCostFunction;
using ceres::CENTRAL;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;



//given a point in the target0 frame (including influence of mill displacements), compute the same point in the camera frame

void transformPoint(double angle_axis[3], double target_tx[3], double point_wrt_target0_frame[3], double *point_wrt_camera_frame) {
    ceres::AngleAxisRotatePoint(angle_axis, point_wrt_target0_frame, point_wrt_camera_frame); //rotate
    point_wrt_camera_frame[0] += target_tx[0]; //then translate
    point_wrt_camera_frame[1] += target_tx[1];
    point_wrt_camera_frame[2] += target_tx[2]; //now have the target point expressed in the camera frame, given transform angle_axis and target_tx translation
    //cout << "transformPoint: point_wrt_camera_frame: " << point_wrt_camera_frame[0] << "," << point_wrt_camera_frame[1] << ", " << point_wrt_camera_frame[2] << endl;
    //cout <<"vector cam to target0 frame: "<<target_tx[0]<<", "<<target_tx[1]<<", "<<target_tx[2]<<endl;

}

//given all data, compute the residuals for point projection w/ provided intrinsics, extrinsics, rotation, and calibration data
//return results in residuals[0],residuals[1];
void compute_residuals(const double * intrinsics, const double *extrinsics, const double *ax_ay_az, double pixel_i, double pixel_j, double target_x, double target_y,
        double sled_dx, double sled_dy, double sled_dz, double *residuals) {
    

        const double fx = intrinsics[0];
        const double fy = intrinsics[1]; /** focal length y */
        const double cx = intrinsics[2]; /** central point x */
        const double cy = intrinsics[3]; /** central point y */
        const double k1 = intrinsics[4]; /** distortion k1  */
        const double k2 = intrinsics[5]; /** distortion k2  */
        const double k3 = intrinsics[6]; /** distortion k3  */
        const double p1 = intrinsics[7]; /** distortion p1  */
        const double p2 = intrinsics[8]; /** distortion p2  */
        cout<<"intrinsics: fx, fy, cx, cy: "<<fx<<", "<<fy<<", "<<cx<<", "<<cy<<endl;
        cout<<"k1,k2,k3,p1,p2: "<<k1<<", "<<k2<<", "<<k3<<", "<<p1<<", "<<p2<<endl;    
        //6DOF extrinsics, target0 frame w/rt camera frame:
        double angle_axis[3], cam_frame_to_target_frame_translation[3];
        for (int i = 0; i < 3; i++) {
            angle_axis[i] = extrinsics[i + 3];
            cam_frame_to_target_frame_translation[i] = extrinsics[i];
        }
        cout <<"compute_residuals(): rotation target frame w/rt cam frame: "<<angle_axis[0]<<", "<<angle_axis[1]<<", "<<angle_axis[2]<<endl;
        cout <<"compute_residuals(): vector cam to target0 frame: "<<cam_frame_to_target_frame_translation[0]<<", "<<cam_frame_to_target_frame_translation[1]<<", "<<cam_frame_to_target_frame_translation[2]<<endl;
        cout <<"compute_residuals(): rot mill frame to target frame ax_ay_az: "<<ax_ay_az[0]<<", "<<ax_ay_az[1]<<", "<<ax_ay_az[2]<<endl;
         // 1. Estimating the axis of motion (relative to initial target pose)
        double nominal_axis_x[3]; // Nominally we move along camera x, y and z axes
        double nominal_axis_y[3];
        double nominal_axis_z[3];

        nominal_axis_x[0] = 1;
        nominal_axis_x[1] = 0;
        nominal_axis_x[2] = 0;

        nominal_axis_y[0] = 0;
        nominal_axis_y[1] = 1;
        nominal_axis_y[2] = 0;

        nominal_axis_z[0] = 0;
        nominal_axis_z[1] = 0;
        nominal_axis_z[2] = 1;

        double rotation_axis[3]; // Here we skew the axes of motion w/ 3 rotation params
        rotation_axis[0] = ax_ay_az[0];
        rotation_axis[1] = ax_ay_az[1];
        rotation_axis[2] = ax_ay_az[2]; //T(0.0);

        //three axes of the mill, expressed in the target frame, renamed for approx
        // alignment between mill axes and target axes (and camera axes)
        double motion_axis_x[3];
        double motion_axis_y[3];
        double motion_axis_z[3];
        //[ 1  -dtz dty
        //  dtz  1 -dtx
        // -dty dtx  1 ]
        motion_axis_x[0] = 1.0; // cos(dtheta_x)
        motion_axis_x[1] = ax_ay_az[2]; //sin(dtheta_z)
        motion_axis_x[2] = -ax_ay_az[1]; //-sin(dtheta_y)

        motion_axis_y[0] = -ax_ay_az[2]; // -sin(dtheta_z)
        motion_axis_y[1] = 1; // cos(dtheta_y)
        motion_axis_y[2] = ax_ay_az[0]; //sin(dtheta_x) rotates y-axis into x direction


        motion_axis_z[0] = ax_ay_az[1]; // sin(dtheta_y)
        motion_axis_z[1] = -ax_ay_az[0]; // -sin(dtheta_x)
        motion_axis_z[2] = 1; //cos(dtheta_z)


        // 2. Move the target point with the mill; express this in "world" frame, which is the target frame at 0_0_0 pose
        //generalize...silly long-hand version of matrix multiply for R*dp
        //wsn: really I am expressing this in a frame defined based on zero mill displacement of target
        // target axes along rows/cols of circles, and target-z normal to the target plane
        //  call this target0 frame
        double point_wrt_target0_frame[3]; /** point in target0 frame */
        //cout << "pixel_i, pixel_j = " << pixel_i_ << ", " << pixel_j_ << endl;
        //cout << "target_x, target_y = " << target_x_ << ", " << target_y_ << endl;
        //cout << "sled dx, dy, dz  = " << sled_dx_ << ", " << sled_dy_ << ", " << sled_dz_ << endl;
        point_wrt_target0_frame[0] = target_x + sled_dx * motion_axis_x[0] + sled_dy * motion_axis_y[0] + sled_dz * motion_axis_z[0];
        point_wrt_target0_frame[1] = target_y + sled_dx * motion_axis_x[1] + sled_dy * motion_axis_y[1] + sled_dz * motion_axis_z[1];
        point_wrt_target0_frame[2] = 0        + sled_dx * motion_axis_x[2] + sled_dy * motion_axis_y[2] + sled_dz * motion_axis_z[2];
        //cout << "WsnCostFunctor: point_wrt_target0_frame: " << point_wrt_target0_frame[0] << "," << point_wrt_target0_frame[1] << ", " << point_wrt_target0_frame[2] << endl;

        /** transform point in target0 coordinates into camera frame */
        double point_wrt_camera_frame[3]; /** point in camera coordinates */
        //transformPoint(double angle_axis[3], double target_tx[3], double point_wrt_target0_frame[3], double &point_wrt_camera_frame[3]) {
        //cout <<"operator(): vector cam to target0 frame: "<<cam_frame_to_target_frame_translation[0]<<", "<<cam_frame_to_target_frame_translation[1]<<", "<<cam_frame_to_target_frame_translation[2]<<endl;

        transformPoint(angle_axis, cam_frame_to_target_frame_translation, point_wrt_target0_frame, point_wrt_camera_frame);
        //cout << "WsnCostFunctor: point_wrt_camera_frame: " << point_wrt_camera_frame[0] << "," << point_wrt_camera_frame[1] << ", " << point_wrt_camera_frame[2] << endl;

        /** compute projection of this point onto image plane and compute residual */
        //double ox = T(ox_);
        //T oy = T(oy_);
        //point in world frame:
        double xp1 = point_wrt_camera_frame[0];
        double yp1 = point_wrt_camera_frame[1];
        double zp1 = point_wrt_camera_frame[2];

        //cout << "WsnCostFunctor; xp1, yp1, zp1 = " << xp1 << "," << yp1 << "," << zp1 << endl << endl;

        /** scale into the image plane by distance away from camera */
        double xp;
        double yp;
        xp = xp1 / zp1;
        yp = yp1 / zp1;


        /* temporary variables for distortion model */
        double xp2 = xp * xp; /* x^2 */
        double yp2 = yp * yp; /* y^2 */
        double r2 = xp2 + yp2; /* r^2 radius squared */
        double r4 = r2 * r2; /* r^4 */
        double r6 = r2 * r4; /* r^6 */

        /* apply the distortion coefficients to refine pixel location */
        double xpp = xp + k1 * r2 * xp // 2nd order term
                + k2 * r4 * xp // 4th order term
                + k3 * r6 * xp // 6th order term
                + p2 * (r2 + 2.0 * xp2) // tangential
                + p1 * xp * yp * 2.0; // other tangential term
        double ypp = yp + k1 * r2 * yp // 2nd order term
                + k2 * r4 * yp // 4th order term
                + k3 * r6 * yp // 6th order term
                + p1 * (r2 + 2.0 * yp2) // tangential term
                + p2 * xp * yp * 2.0; // other tangential term

        /** perform projection using focal length and camera center into image plane */
        // PROVIDE THE RESIDUALS:
        double computed_pixel_i,computed_pixel_j;
        computed_pixel_i = fx * xpp + cx;
        computed_pixel_j = fy * ypp + cy;
        residuals[0] = computed_pixel_i - pixel_i;
        residuals[1] = computed_pixel_j - pixel_j;
        double sqrd_err = residuals[0]*residuals[0]+residuals[1]*residuals[1];
        //cout<<"computed_pixel_i, computed_pixel_j = "<<computed_pixel_i<<", "<<computed_pixel_j<<endl;
        //cout<<"observed pixel_i, pixel_j = "<<pixel_i_<<", "<<pixel_j_<<endl;
        cout<<"residuals: "<<residuals[0]<<", "<<residuals[1]<<endl<<endl;
        cout<<"r2, sqrd_err: "<<r2<<", "<<sqrd_err<<endl;
}

double compute_rms_error(double *intrinsics, double *extrinsics, double *ax_ay_az,
           std::vector<Eigen::Vector2d> xy_pixels_vec,std::vector<Eigen::Vector2d> xy_targets_vec,  std::vector<Eigen::Vector3d> xyz_sled_vec ) {
    double rms_err=0;
    double sum_sqrd_err=0.0;
    double sqrd_err=0;
    
    Eigen::Vector2d xy_pixels, xy_targets;
    Eigen::Vector3d xyz_sled;         
    double residuals[2];

    int nlines = (int) xy_pixels_vec.size();
    for (int i = 0; i < nlines; i++) {
        xy_pixels = xy_pixels_vec[i];
        xy_targets = xy_targets_vec[i];
        xyz_sled = xyz_sled_vec[i];    
        compute_residuals(intrinsics, extrinsics, ax_ay_az, xy_pixels[0], xy_pixels[1], xy_targets[0], xy_targets[1], xyz_sled[0], xyz_sled[1], xyz_sled[2], residuals); 
        sqrd_err = residuals[0]*residuals[0]+residuals[1]*residuals[1];
        sum_sqrd_err+= sqrd_err;
    }
    return rms_err= sqrt(sum_sqrd_err/nlines);
}

//the following class defines an operator() that changes values of the parameters in its argument list
// initialize these values to seed values
//operator should compute/return the residual values
// operator must also have access to change the parameter values, though that is achieved through automated gradient search via Ceres

class WsnCostFunctor {
public:
    //use as follows:
    // WsnCostFunctor(const double* const pixel_i, const double* const pixel_j,const double* const target_x, const double* const target_y, const double* const sled_dx, const double* const sled_dy, const double* const sled_dz) // : pixel_i_(pixel_i), pixel_j_(pixel_j),target_x_(target_x),target_y_(target_y),sled_dx_(sled_dx),sled_dy_(sled_dy),sled_dz_(sled_dz) {}
    // constructor should have arguments for fixed inputs (calibration data points)
    //WsnCostFunctor(const double* const pixel_i, const double* const pixel_j,const double* const target_x, const double* const target_y, const double* const sled_dx, const double* const sled_dy, const double* const sled_dz) // : pixel_i_(pixel_i), pixel_j_(pixel_j),target_x_(target_x),target_y_(target_y),sled_dx_(sled_dx),sled_dy_(sled_dy),sled_dz_(sled_dz) {}
 
    //in this version, do not use pointers; DO use initializers to set values of class member vars
    WsnCostFunctor(double pixel_i, double pixel_j,double target_x, double target_y, double sled_dx, double sled_dy, double sled_dz) : pixel_i_(pixel_i), pixel_j_(pixel_j),target_x_(target_x),target_y_(target_y),sled_dx_(sled_dx),sled_dy_(sled_dy),sled_dz_(sled_dz) {}
    
    //here is the operator() that must input modifiable params and provide residuals
  
    bool operator()(const double* intrinsics, const double* extrinsics, const double* ax_ay_az, double* residuals) const {
        //the real work is done here: given intrinsics and extrinsics, compute the expected projection of a point onto pixel space
        //compare this to the corresponding pixel-space data point; discrepancies are residuals (i and j, separately)
        compute_residuals(intrinsics, extrinsics, ax_ay_az, pixel_i_,  pixel_j_,  target_x_,  target_y_,
               sled_dx_,  sled_dy_,  sled_dz_, residuals);
        return true;
    } //end of operator() definition

private:
    const double pixel_i_, pixel_j_, target_x_, target_y_, sled_dx_, sled_dy_, sled_dz_; //private member vars set by initializers in class constructor


};  //end of class WsnCostFunctor definition



int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);


    //calibration input data; do not vary these
    std::vector<Eigen::Vector2d> xy_pixels_vec;
    std::vector<Eigen::Vector2d> xy_targets_vec;
    std::vector<Eigen::Vector3d> xyz_sled_vec;
    Eigen::Vector2d xy_pixels, xy_targets;
    Eigen::Vector3d xyz_sled;
    //read the calibration file:
    std::string fname("calibration_points.txt");
    if (!read_calibration_file(fname, xy_pixels_vec, xy_targets_vec, xyz_sled_vec)) {
        cout << "could not open file " << fname << "; quitting" << endl;
        return 1;
    }
    cout << "calibration file has been read in" << endl;
    xy_pixels = xy_pixels_vec[0];
    xy_targets = xy_targets_vec[0];
    xyz_sled = xyz_sled_vec[0];
    //sanity check on reading calibration file:
    cout << "first row: " << xy_pixels[0] << "," << xy_pixels[1] << "," << xy_targets[0] << "," << xy_targets[1] << "," << xyz_sled[0] << "," << xyz_sled[1] << "," << xyz_sled[2] << endl;


    // Build the problem.
    Problem problem;

    double image_x, image_y, target_x, target_y, sled_x, sled_y, sled_z; //these values are all known, corresponding to a calibration data point
    double intrinsics[9], extrinsics[6], ax_ay_az[3]; //these values are all parameters to be discovered:
    //intrinsics:
    //	     focal_length_x,
    //       focal_length_y,
    //	      center_x,
    //	      center_y,
    //	      distortion_k1,
    //	      distortion_k2,
    //	      distortion_k3,
    //	      distortion_p1,
    //	      distortion_p2;

    //initial guesses for parameters:
    intrinsics[0] = 450; //guess at the focal length, in pixels
    intrinsics[1] = 450;
    intrinsics[2] = 704 / 2; //704x480
    intrinsics[3] = 480 / 2;
    for (int i = 4; i < 9; i++) intrinsics[i] = 0.0; //init distortion to zero
    for (int i = 0; i < 3; i++) ax_ay_az[i] = 0.0; //init approx rotation from mill frame to target frame = 0 angle/axis

    extrinsics[0] = 0.1; //scene of image_0_0_0.jpg has target origin about 0.1m to right of camera frame, but about centered up/down
    extrinsics[1] = 0.0; //target origin is about centered up/down for first scene
    extrinsics[2] = 0.3; //not sure how far target is from camera at closest approach
    for (int i = 3; i < 6; i++) extrinsics[i] = 0.0; //target and camera frames are approximately aligned

        /*
  for (int i = 0; i < kNumObservations; ++i) {
    problem.AddResidualBlock(
        new AutoDiffCostFunction<ExponentialResidual, 1, 1, 1>(
            new ExponentialResidual(data[2 * i], data[2 * i + 1])),
        NULL,
        &m, &c);
  }         
         */
    int nlines = (int) xy_pixels_vec.size();
    for (int i = 0; i < nlines; i++) {
        xy_pixels = xy_pixels_vec[i];
        xy_targets = xy_targets_vec[i];
        xyz_sled = xyz_sled_vec[i];

        //CostFunction *cost_function = RailICal5::Create(xy_pixels[0],xy_pixels[1],xyz_sled[0],xyz_sled[1],xyz_sled[2],point);

        //    WsnCostFunctor(double pixel_i, double pixel_j,double target_x, double target_y, double sled_dx, double sled_dy, double sled_dz) : pixel_i_(pixel_i), pixel_j_(pixel_j),target_x_(target_x),target_y_(target_y),sled_dx_(sled_dx),sled_dy_(sled_dy),sled_dz_(sled_dz) {}
        //CostFunction* cost_function =
        //  new NumericDiffCostFunction<Rat43CostFunctor, FORWARD, 1, 4>(
        //      new Rat43CostFunctor(x, y));
        cout << "row " << i << ": " << xy_pixels[0] << "," << xy_pixels[1] << "," << xy_targets[0] << "," << xy_targets[1] << "," << xyz_sled[0] << "," << xyz_sled[1] << "," << xyz_sled[2] << endl;
        problem.AddResidualBlock(
            new NumericDiffCostFunction<WsnCostFunctor, CENTRAL, 2, 9, 6, 3>(
                new WsnCostFunctor(xy_pixels[0], xy_pixels[1], xy_targets[0], xy_targets[1], xyz_sled[0], xyz_sled[1], xyz_sled[2])),
                    NULL,
                    intrinsics, extrinsics, ax_ay_az);
    }
    cout << "done adding residuals; call the solver..."<<endl;
    // Run the solver!
    Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.max_num_iterations = 1000;    

    Solver::Summary summary;
    Solve(options, &problem, &summary);

    double rms_err = compute_rms_error(intrinsics, extrinsics, ax_ay_az,xy_pixels_vec,xy_targets_vec, xyz_sled_vec );
    cout<<"rms_err = "<<rms_err<<endl;
    /*
    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x
              << " -> " << x << "\n"; */
    return 0;
}
