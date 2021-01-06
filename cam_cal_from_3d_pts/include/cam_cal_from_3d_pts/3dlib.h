#include <Eigen/Eigen>

namespace cal{
	Eigen::Affine3d cam_cal(
		const std::string points_3d,
		const std::string points_uv,
		const double fx, const double fy, const double cx, const double cy,
		const Eigen::Affine3d & seed,
		double & accuracy
	);
}
