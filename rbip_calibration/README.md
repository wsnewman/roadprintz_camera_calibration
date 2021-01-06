# rbip_calibration
programs for camera-based painting calibration.  



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

## Running tests/demos
    
