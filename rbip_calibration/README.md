# rbip_calibration
programs for camera-based painting calibration.  
Initial focus is on reconciling vision with laser pointing



10/25/20:
new program: find_extrinsics
uses correspondences between LIDAR-based poster corners and snapshot poster corners
used 7 files containing 1008 points.
Found extrinsics fit to 4.4 pixels rms
BUT transforms do not make sense

1/4/2021:
added library openCV_utils; currently only has one function: find_poster()
see corresponding test main, openCV_utils_example_main.cpp


## Example usage
test laser-pointing calibration (uses small checkerboard)
start up Stella and move arm to camera pose.  Run these additional nodes:
`rosrun rbip_calibration cam_poster_calib_test_service_node`
  (this node bridges user and motion commands; performs image capture and image process)

`rosrun test_moves test_laser_moves_RBIP_frame`
   (this node/service receives commands of (x,y) and causes robot to move safely to point laser at this point)
    
`rosrun rbip_calibration cam_poster_calib_test_main`
   (this is a crude, text user interface, to be replaced w/ GUI buttons)

for simu testing, see more detail in: instructions/how_to_run_vision_calib_test_w_small_poster
