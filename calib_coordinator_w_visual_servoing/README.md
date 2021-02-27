# calib_coordinator_w_visual_servoing
This is the top-level program for acquiring camera-calibration data.


## Example usage
With Intel camera mounted to toolflange (in PROPER ORIENTATION!!)
and using 10x7 poster

Start up Stella

put robot in camera pose

`roslaunch realsense2_camera rp_rs_camera.launch`

`rosrun test_moves l515_move_service`

`rosrun pcl_utils make_image_from_cloud_service`

`rosrun rbip_calibration find_poster_in_lidar_image_10x7_service`

`rosrun calib_coordinator_w_visual_servoing calib_coordinator_w_visual_servoing`

Coordinator will prompt user for inputs (root name for files, including E-stop, moving poster, acquiring LIDAR scan, ...)

Look for recorded data in: ROS_WS/calibration_temp_files/

## Running tests/demos
    
