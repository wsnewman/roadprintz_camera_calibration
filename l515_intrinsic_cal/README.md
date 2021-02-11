# l515_intrinsic_cal

coordinator for moving robot and taking snapshots with Intel L515 sensor attached to toolflange.
Data will be written to directory $ROS_WS/stella_calib_data/l515_intrinsic_data/

Output file: pose_info.txt 
 will explain how to interpret the x, y, z increments in terms of the image filenames

Will need to edit parameters for X_MIN, X_MAX, DX, etc to specify the range of desired views and
number of snapshots to acquire.

Can use this node to help tune these parameters:
`rosrun test_moves test_main_l515_move_service`
which works in conjuction with:
`rosrun test_moves l515_move_service`
and:
`roslaunch realsense2_camera rs_camera.launch` 

After establishing values for coordinator, run the coordinator as follows:

run this node after starting up robot and L515 driver:
`roslaunch realsense2_camera rs_camera.launch` 

Make sure camera images are being published on expected camera topic 
 make sure this topic matches topic set in coordinator code:
    string g_input_image_topic="/camera/image_raw";

move robot to home position

Start the coordinator and move service nodes:

`rosrun test_moves l515_move_service`
 
`rosrun l515_intrinsic_cal l515_intrinsic_calib_data_acq_coordinator`

coordinator will prompt to enable motion to middle pose of sequence, and inform user to E-stop the robot.
Place the small poster under the robot, roughly centered in the camera view

Respond to coordinator prompts to re-enable the robot and start data acquisition

post-process the images in the l515_intrinsic_data/ directory to obtain the camera's intrinsic properties

(still need to obtain the extrinsic camera-mount properties)


    