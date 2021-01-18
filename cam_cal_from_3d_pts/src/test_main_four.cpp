#include <iostream>

#include "yaml-cpp/yaml.h"

#include <cam_cal_from_3d_pts/3dlib.h>

int main(int argc, char ** argv){

	//Read in intrinsics:
	YAML::Node intrinsic_file;
	try{
		intrinsic_file = YAML::LoadFile(argv[3]);
	} catch(YAML::BadFile e){//If file is not extant and well-formed...
		printf(
			"\e[39mCamera intrinsics file \"%s\" does not exist or contains syntax errors.\e[31m\n",
			argv[3]
		);
		return 0;
	}
	double fx, fy, cx, cy;
	//Rectified images so no need for distortion.
	try{
		fx = intrinsic_file["fx"].as<double>();
		fy = intrinsic_file["fy"].as<double>();
		cx = intrinsic_file["cx"].as<double>();
		cy = intrinsic_file["cy"].as<double>();
	} catch(YAML::RepresentationException e){
		printf("\e[39mIntrinsic parse exception \"%s\".\e[31m\n", e.what());
		return 0;
	}
	printf(
		"\nSuccessfully initialized intrinsics from %s:\n",
		argv[3]
	);
	printf("\tfx:%f\n\tfy:%f\n\tcx:%f\n\tcy:%f\n\n", fx, fy, cx, cy);
	
	
	//Init the transform.
	Eigen::Affine3d init_affine;
	//Taken from simulation camera pose w.r.t. system base frame.
	init_affine.matrix() <<
/*	 -0.0565683,   -0.998263,   0.0164475,    -1.00925,
	   -0.99837,   0.0566835,  0.00662688,       1.038,
	-0.00754767,  -0.0160458,   -0.999843,    -2.80553,
	          0,           0,           0,           1*/
	;
	
	
	//Run the thing!
	double acc;
	Eigen::Affine3d a = cal::cam_cal_four(
		std::string(argv[1]),
		std::string(argv[2]),
		fx, fy, cx, cy,
		init_affine,
		acc
	);
	
	std::printf("\n\n");
	std::cout << a.matrix() << "\n\n";
	std::printf("%f\n\n", acc);

	return 0;
}
