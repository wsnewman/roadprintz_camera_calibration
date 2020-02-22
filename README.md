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










