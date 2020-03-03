# roadprintz_camera_calib	
Code derived/modified from ros-industrial camera calibration here:
https://github.com/ros-industrial/industrial_calibration/tree/kinetic-devel/industrial_extrinsic_cal

run this from the directory containing the calibration snapshots of interest.
E.g., see subfolder /images/huawei_ipcam_calib_snapshots

NOTE: at present, this node has hard-coded names for the images, e.g.:
 image_0_0_0.jpg, which is the reference starting pose on the mill (lower elevation, full right, full forward).
All other images are image_x_y_z.jpg, where x  value is index --> 100mm*index (to the left), y value is index --> 100mm*index (front to back) 
and z value is index--> z=100mm*index bottom to top.  Thus, (precision) relative displacement is encoded in the image name.

target is from:
https://github.com/ros-industrial/industrial_calibration/tree/kinetic-devel/industrial_extrinsic_cal/targets

roscd roadprintz_camera_calibration
cd images/huawei_ipcam_calib_snapshots
(roscore)
rosrun roadprintz_camera_calibration  test_find_patterns

this code has lots of debug/diagnostic output, including checkpoints.

There are several scenes for which circle detection fails and/or pattern detection fails--even when all circles are found.

update 3/1/20:
roscd roadprintz_camera_calibration
cd images/huawei_ipcam_calib_snapshots
rosrun roadprintz_camera_calibration test_find_patterns 

runs slowly, and creates output file: calibration_points.txt
 this contains the data in the form: i_pixel, j_pixel, x_target_frame, y_target_frame, dx_mill, dy_mill, dz_mill
 in source code, deliberately chose to redefine mill sled axes to correspond to (very near to) target axes, x,y,z
 still need to find the rotation transform from mill orientation to (circle) target orientation, but should be a small perturbation
  of dtheta_x, dtheta_y, dtheta_z

 post-process the calibration-data file with ceres.
 see: wsn_ceres_notes in ~/ceres-solver-1.14.0 (in which I added a new executable as part of the "examples" directory)














