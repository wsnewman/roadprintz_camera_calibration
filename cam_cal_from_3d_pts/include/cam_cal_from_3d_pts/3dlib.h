#include <Eigen/Eigen>

namespace cal{
	Eigen::Affine3d cam_cal(
		const std::string points_3d,
		const std::string points_uv,
		const double fx, const double fy, const double cx, const double cy,
		const Eigen::Affine3d & seed,
		double & accuracy
	);
	
	Eigen::Affine3d cam_cal_four(
		const std::string points_3d,
		const std::string points_uv,
		const double fx, const double fy, const double cx, const double cy,
		const Eigen::Affine3d & seed,
		double & accuracy
	);
	
	Eigen::Affine3d match_points(
		const std::vector<Eigen::Vector2d> points_3d,
		const std::vector<Eigen::Vector2d> points_uv,
		const double fx, const double fy, const double cx, const double cy,
		const Eigen::Affine3d & seed,
		double & accuracy
	);
		
}
