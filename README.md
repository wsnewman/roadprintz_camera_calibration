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


update 3/18/20:
had to re-do rectification, since x and y scales were different.
Do this with:  rosrun ip_cam ipcam_fake_huawei_driver, then enter image of interest to rectify;
 then run rqt_image_view, subscribe to the rectified image, then save this rectified image to disk

opened imag17_rect.png w/ GIMP; corners are at:
(307,200)-->(417,370)
dj = (370-200 )/12 = 14.166
di = ( 417-307 )/7 = 15.7
NOT yet rectified

redo w/ image17_rect and image17_rescaled
rect: (306,200) -> (417,370)


rescaled: corners: (307,220)->(417,407)
di = (417-307)/7 = 15.71
dj = (407-220)/12 = 15.58 (OK)

rescaled version looks better;

Then run:
rosrun roadprintz_camera_calibration analyze_image17
click on corners:
Left button of the mouse is clicked - position (308, 220)
upper left corner set to 308, 220
Left button of the mouse is clicked - position (416, 406)
lower-right corner set to 416, 406

di = (416-308)/7  = 15.43
dj = (406-220)/12 = 15.5
(OK)

hard-code corner coords to overlay grid:

 
10/25/20:
new program: find_extrinsics
uses correspondences between LIDAR-based poster corners and snapshot poster corners
used 7 files containing 1008 points.
Found extrinsics fit to 4.4 pixels rms
BUT transforms do not make sense















