To test library, `roslaunch cam_cal_from_3d_pts test_four.launch`. This will run a test main that executes `cam_cal_four` and `match_points`.


Data goes in the `/data` folder. ATM, you have to edit the launch file to change the file names. By default they are `image_key_pts_metric.csv` and `image_key_pts_pixels.csv`.


~~~
Eigen::Affine3d cam_cal_four(
	const std::string points_3d,
	const std::string points_uv,
	const double fx, const double fy, const double cx, const double cy,
	const Eigen::Affine3d & seed,
	double & accuracy
);
~~~
Reads in two .csv files and **sorts** the image data into a specific order; then runs `match_points` on them.


~~~
Eigen::Affine3d match_points(
	const std::vector<Eigen::Vector2d> points_3d,
	const std::vector<Eigen::Vector2d> points_uv,
	const double fx, const double fy, const double cx, const double cy,
	const Eigen::Affine3d & seed,
	double & accuracy
);
~~~
Given two generic paired lists of points and finds an affine in between them.


Note that this is a **static** library; there is no object that needs to be created to run these.


Needs [roadprintz_cam_cal_tes](https://github.com/RoadPrintz/roadprintz_cam_cal_tes) for cc_utils lib.
