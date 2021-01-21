//#include "ceres/ceres.h"

#include <fstream>

#include <cc_utils/cc_utils.h>
#include <cam_cal_from_3d_pts/3dlib.h>

#define VERBOSE

namespace cal{
	class CalibrationEntry{
		public:
		//Giant stupid constructor chain.
		//The fact that it takes two functions to pass the exact same arguments into the exact same variables is Ceres' fault; not mine.
		CalibrationEntry(
			const double image_pixels_in[2],
			
			const double TARGET_POINT_in[3],
			
			const double f_x_in, const double f_y_in, const double c_x_in, const double c_y_in
		){
			image_pixels[0] = image_pixels_in[0];
			image_pixels[1] = image_pixels_in[1];
			
			TARGET_POINT[0] = TARGET_POINT_in[0];
			TARGET_POINT[1] = TARGET_POINT_in[1];
			TARGET_POINT[2] = TARGET_POINT_in[2];
			
			f_x = f_x_in;
			f_y = f_y_in;
			c_x = c_x_in;
			c_y = c_y_in;
		}
		static ceres::CostFunction* Create(
			const double image_pixels_in[2],
			
			const double TARGET_POINT_in[3],
			
			const double f_x_in, const double f_y_in, const double c_x_in, const double c_y_in
		){
			return new ceres::AutoDiffCostFunction<CalibrationEntry, 2,//Residual output comes first.
				//	translation	rotation
				3,			3
			>(new CalibrationEntry (//Just pass all the arguments in in the same order.
				image_pixels_in,
			
				TARGET_POINT_in,
				
				f_x_in, f_y_in, c_x_in, c_y_in
			));
		}
	
	
		//Constant member vars
		//Perpoints
		double image_pixels[2];
		double TARGET_POINT[3];
		//Globals (which need to be passed perpoint anyway)
		double f_x, f_y, c_x, c_y;
		
		
		template<typename T> bool operator()(//TODO Why are all these const / should all these be const?
			const T* BASE_to_CAM_translation, const T* BASE_to_CAM_rotation,
			
			T* residual
		) const {
		
			//CAM_to_POINT = CAM_to_BASE * BASE_to_POINT	
			T BASE_to_POINT [3] = {
				T(TARGET_POINT[0]),
				T(TARGET_POINT[1]),
				T(TARGET_POINT[2]),
			};
			
			/*std::cout << "BASE TO POINT:\n";
			std::cout << BASE_to_POINT[0] << "\n";
			std::cout << BASE_to_POINT[1] << "\n";
			std::cout << BASE_to_POINT[2] << "\n\n";*/
			
			T CAM_to_POINT [3];
			cc_utils::transformPoint_euler(BASE_to_CAM_translation, BASE_to_CAM_rotation, BASE_to_POINT, CAM_to_POINT);
			
			/*std::cout << "CAM TO POINT:\n";
			std::cout << CAM_to_POINT[0] << "\n";
			std::cout << CAM_to_POINT[1] << "\n";
			std::cout << CAM_to_POINT[2] << "\n";*/
			
			T u, v;
			cc_utils::project(
				CAM_to_POINT[0], CAM_to_POINT[1], CAM_to_POINT[2],
				
				T(f_x), T(f_y), T(c_x), T(c_y),
				T(0), T(0), T(0), T(0), T(0),//No distortion in rectified image.
			
				T(0), T(0),//TODO Get rid of these.
			
				u, v
			);
			
			/*printf("\nReal u, v: (%f, %f)\n", image_pixels[0], image_pixels[1]);
			printf("Calc u, v: (%f, %f)\n", cc_utils::val(u), cc_utils::val(v));*/
			
			cc_utils::add_to_visualization(cc_utils::val(u), cc_utils::val(v), image_pixels[0], image_pixels[1]);
			
			//std::getchar();
			
			residual[0] = u - T(image_pixels[0]);
			residual[1] = v - T(image_pixels[1]);
			
			return true;
		}
	};

	Eigen::Affine3d cam_cal(
		const std::string points_3d,
		const std::string points_uv,
		const double fx, const double fy, const double cx, const double cy,
		const Eigen::Affine3d & seed,
		double & accuracy
	){
		std::ifstream data_file;
		std::vector<Eigen::Vector3d> vec_3d;
		std::vector<Eigen::Vector2d> vec_uv;
		
		
		std::vector<Eigen::Vector2d> vec_uv_acc;
		data_file.open(points_uv);
		if(!data_file){
			printf("\e[39mCould not find px data file \"%s\".\e[31m\n", points_uv.c_str());
			accuracy = -1.0;
			Eigen::Affine3d a;
			return a;
		}
		int n = 0;
		char line [255];
		while(data_file.getline(line, 255)){
			n++;
			std::string line_s = std::string(line);
			if(std::count(line_s.begin(), line_s.end(), ',') != 1){
				printf("\e[39mBad file format. Line %d (%s).\e[31m\n", n, line);
				accuracy = -1.0;
				Eigen::Affine3d a;
				return a;
			}
		
			Eigen::Vector2d pixel_location;
		
			sscanf(line, "%lf, %lf",
				&pixel_location.x(), &pixel_location.y()
			);
			vec_uv_acc.push_back(pixel_location);
		}
		data_file.close();
		#ifdef VERBOSE
		printf("\nRead in \e[1m%d\e[0m entries from %s.\n", n, points_uv.c_str());
		#endif
		
		if(n % 144 != 0){
			printf("\e[39m%d is not an even number of target points. They should be a multiple of 144\e[31m\n", n);
			accuracy = -1.0;
			Eigen::Affine3d a;
			return a;
		}
		
		int t = n / 144;
		#ifdef VERBOSE
		printf("\nThis is \e[1m%d\e[0m targets.\n\n", t);
		#endif
		
		std::vector<std::vector<Eigen::Vector2d> > vec_uv_segment(t);
		int count = 0;
		for(int i = 0; i < t; i++){
			std::vector<Eigen::Vector2d> component(144);
			for(int j = 0; j < 144; j++){
				component[j] = vec_uv_acc[count];
				count++;
			}
			vec_uv_segment[i] = component;
		}
		
		std::vector<Eigen::Vector3d> vec_3d_acc;
		data_file.open(points_3d);
		if(!data_file){
			printf("\e[39mCould not find 3d data file \"%s\".\e[31m\n", points_3d.c_str());
			accuracy = -1.0;
			Eigen::Affine3d a;
			return a;
		}
		while(data_file.getline(line, 255)){
			std::string line_s = std::string(line);
			if(std::count(line_s.begin(), line_s.end(), ',') != 1){
				printf("\e[39mBad file format. Line %d (%s).\e[31m\n", n, line);
				accuracy = -1.0;
				Eigen::Affine3d a;
				return a;
			}
		
			Eigen::Vector3d metric_location;
		
			sscanf(line, "%lf, %lf",
				&metric_location.x(), &metric_location.y()
			);
			metric_location.z() = 0;
			vec_3d_acc.push_back(metric_location);
		}
		data_file.close();
		#ifdef VERBOSE
		printf("\nRead in \e[1m%lu\e[0m entries from %s.\n\n", vec_3d_acc.size(), points_3d.c_str());
		#endif
		
		if(vec_3d_acc.size() % 4 != 0){
			printf("\e[39m%lu is not an even number of edge points. They should be a multiple of 4\e[31m\n", vec_3d_acc.size());
			accuracy = -1.0;
			Eigen::Affine3d a;
			return a;
		}
		
		if(vec_3d_acc.size() / 4 != t){
			printf("\e[39m%lu is not the right number of targets. There should be %d.\e[31m\n", vec_3d_acc.size(), t);
			accuracy = -1.0;
			Eigen::Affine3d a;
			return a;
		}
		
		std::vector<std::vector<Eigen::Vector3d> > vec_3d_segment(t);
		count = 0;
		for(int i = 0; i < t; i++){
			std::vector<Eigen::Vector3d> component(4);
			for(int j = 0; j < 4; j++){
				component[j] = vec_3d_acc[count];
				count++;
			}
			vec_3d_segment[i] = component;
		}
		
		std::vector<std::vector<Eigen::Vector3d> > vec_3d_fulls(t);
		for(int i = 0; i < t; i++){
			Eigen::Vector3d tl, tr, bl, br;
			std::vector<Eigen::Vector3d> candidates = vec_3d_segment[i];
			
			int toppest_id = 0;
			int bottomest_id = 0;
			
			for(int j = 0; j < 4; j++){
				if(candidates[j].y() < candidates[toppest_id].y()){
					toppest_id = j;
				}
				if(candidates[j].y() > candidates[bottomest_id].y()){
					bottomest_id = j;
				}
			}
			
			int second_toppest_id = 0;
			int second_bottomest_id = 0;
			for(int j = 0; j < 4; j++){
				if(candidates[j].y() < candidates[second_toppest_id].y() && j != toppest_id){
					second_toppest_id = j;
				}
				if(candidates[j].y() > candidates[second_bottomest_id].y() && j != bottomest_id){
					second_bottomest_id = j;
				}
			}
			
			if(candidates[toppest_id].x() > candidates[second_toppest_id].x()){
				tl = candidates[second_toppest_id];
				tr = candidates[toppest_id];
			} else{
				tl = candidates[toppest_id];
				tr = candidates[second_toppest_id];
			}
			
			if(candidates[bottomest_id].x() > candidates[second_bottomest_id].x()){
				bl = candidates[second_bottomest_id];
				br = candidates[bottomest_id];
			} else{
				bl = candidates[bottomest_id];
				br = candidates[second_bottomest_id];
			}
			
			std::vector<Eigen::Vector3d> full;
			
			double dx_h = tr.x() - tl.x();
			double dy_h = tr.y() - tl.y();
			double dx_v = bl.x() - tl.x();
			double dy_v = bl.y() - tl.y();
			
			for(int is = 0; is < 8; is++){
				double tv = (double)is / 7.0;
			
				double x0 = tl.x() + dx_v * tv;
				double y0 = tl.y() + dy_v * tv;
			
				for(int js = 0; js < 18; js++){
			
					Eigen::Vector3d cp;
			
					double th = (double)js / 17.0;
				
					cp.x() = x0 + dx_h * th;
					cp.y() = y0 + dy_h * th;
					cp.z() = 0.0;
				
					full.push_back(cp);
				}
			}
			vec_3d_fulls[i] = full;
		}
		
		for(int i = 0; i < t; i++){
			for(int j = 0; j < 144; j++){
				vec_uv.push_back(vec_uv_segment[i][j]);
				vec_3d.push_back(vec_3d_fulls[i][j]);
			}
		}
		
		if(vec_3d.size() != vec_uv.size()){
			printf("\e[39mMismatch between calculated points (%lu) and detected points (%d).\e[31m\n", vec_3d.size(), n);
			accuracy = -1.0;
			Eigen::Affine3d a;
			return a;
		}
		#ifdef VERBOSE
		printf("\nCreated %lu target points. This is the correct number!\n\n", vec_3d.size());
		#endif
		
		
		
		Eigen::Vector3d ea = seed.rotation().eulerAngles(2, 1, 0);
   	
		double t_array [3] = {
			seed.translation().x(),
			seed.translation().y(),
			seed.translation().z()
		};
		double r_array [3] = {
			ea.z(),
			ea.y(),
			ea.x()
		};
		
		
		//Build the optimization problem
		ceres::Problem problem;
		ceres::Solver::Options options;
		
		cc_utils::init_visualization(2688, 1520, vec_uv, options);
		
		
		
		for(int i = 0; i < n; i++){
			double pixels_as_array [2] = { vec_uv[i].x(), vec_uv[i].y() };
			double target_as_array [3] = { vec_3d[i].x(), vec_3d[i].y(), vec_3d[i].z() };
		
			ceres::CostFunction *cost_function = CalibrationEntry::Create(
				pixels_as_array,
				target_as_array,
				fx, fy, cx, cy
			);
		
		
			problem.AddResidualBlock(cost_function, NULL,
				t_array, r_array
			);
		}
		cc_utils::bound_rotation(problem, r_array);
		
		#ifdef VERBOSE
		options.minimizer_progress_to_stdout = true;
		#endif
		options.linear_solver_type = ceres::DENSE_SCHUR;
		options.max_num_iterations = 1000;
		ceres::Solver::Summary summary;
    		ceres::Solve(options, &problem, &summary);
	
		Eigen::Affine3d a;
		a =
			Eigen::AngleAxisd(r_array[2], Eigen::Vector3d::UnitZ()) *
   			Eigen::AngleAxisd(r_array[1], Eigen::Vector3d::UnitY()) *
   			Eigen::AngleAxisd(r_array[0], Eigen::Vector3d::UnitX());
   		a.translation() = Eigen::Vector3d(t_array[0], t_array[1], t_array[2]);
   		
   		accuracy = cc_utils::rms();
   		
		return a;
	}
	
	Eigen::Affine3d cam_cal_four(
		const std::string points_3d,
		const std::string points_uv,
		const double fx, const double fy, const double cx, const double cy,
		const Eigen::Affine3d & seed,
		double & accuracy
	){
	
		std::ifstream data_file;
		std::vector<Eigen::Vector2d> vec_3d;
		std::vector<Eigen::Vector2d> vec_uv;
		
		data_file.open(points_uv);
		if(!data_file){
			printf("\e[39mCould not find px data file \"%s\".\e[31m\n", points_uv.c_str());
			accuracy = -1.0;
			Eigen::Affine3d a;
			return a;
		}
		int n = 0;
		char line [255];
		while(data_file.getline(line, 255)){
			n++;
			std::string line_s = std::string(line);
			if(std::count(line_s.begin(), line_s.end(), ',') != 1){
				printf("\e[39mBad file format. Line %d (%s).\e[31m\n", n, line);
				accuracy = -1.0;
				Eigen::Affine3d a;
				return a;
			}
		
			Eigen::Vector2d pixel_location;
		
			sscanf(line, "%lf, %lf",
				&pixel_location.x(), &pixel_location.y()
			);
			vec_uv.push_back(pixel_location);
		}
		data_file.close();
		#ifdef VERBOSE
		printf("\nRead in \e[1m%d\e[0m entries from %s.\n\n", n, points_uv.c_str());
		#endif
		
		//Sort entries in groups of 4
		if(vec_uv.size() % 4 != 0){
			printf("\e[39mThe pixel locations are not in groups of 4. \"%s\".\e[31m\n", points_3d.c_str());
			accuracy = -1.0;
			Eigen::Affine3d a;
			return a;
		}
		for(int i = 0; i < vec_uv.size(); i+=4){
			Eigen::Vector2d tl, tr, bl, br;
			std::vector<Eigen::Vector2d> candidates(4);
			for(int j = 0; j < 4; j++){
				candidates[j] = vec_uv[i+j];
			}
			
			int toppest_id = 0;
			int bottomest_id = 0;
			
			for(int j = 0; j < 4; j++){
				if(candidates[j].y() < candidates[toppest_id].y()){
					toppest_id = j;
				}
				if(candidates[j].y() > candidates[bottomest_id].y()){
					bottomest_id = j;
				}
			}
			
			int second_toppest_id = 0;
			int second_bottomest_id = 0;
			for(int j = 0; j < 4; j++){
				if(candidates[j].y() < candidates[second_toppest_id].y() && j != toppest_id){
					second_toppest_id = j;
				}
				if(candidates[j].y() > candidates[second_bottomest_id].y() && j != bottomest_id){
					second_bottomest_id = j;
				}
			}
			
			if(candidates[toppest_id].x() > candidates[second_toppest_id].x()){
				tl = candidates[second_toppest_id];
				tr = candidates[toppest_id];
			} else{
				tl = candidates[toppest_id];
				tr = candidates[second_toppest_id];
			}
			
			if(candidates[bottomest_id].x() > candidates[second_bottomest_id].x()){
				bl = candidates[second_bottomest_id];
				br = candidates[bottomest_id];
			} else{
				bl = candidates[bottomest_id];
				br = candidates[second_bottomest_id];
			}
			
			vec_uv[i+0] = tl;
			vec_uv[i+1] = bl;
			vec_uv[i+2] = br;
			vec_uv[i+3] = tr;
		}
		
		
		data_file.open(points_3d);
		if(!data_file){
			printf("\e[39mCould not find 3d data file \"%s\".\e[31m\n", points_3d.c_str());
			accuracy = -1.0;
			Eigen::Affine3d a;
			return a;
		}
		while(data_file.getline(line, 255)){
			std::string line_s = std::string(line);
			if(std::count(line_s.begin(), line_s.end(), ',') != 1){
				printf("\e[39mBad file format. Line %d (%s).\e[31m\n", n, line);
				accuracy = -1.0;
				Eigen::Affine3d a;
				return a;
			}
		
			Eigen::Vector2d metric_location;
		
			sscanf(line, "%lf, %lf",
				&metric_location.x(), &metric_location.y()
			);
			vec_3d.push_back(metric_location);
		}
		data_file.close();
		#ifdef VERBOSE
		printf("\nRead in \e[1m%d\e[0m entries from %s.\n\n", n, points_3d.c_str());
		#endif
		
		if(vec_3d.size() != n){
			printf("\e[39mMismatch between calculated points (%lu) and detected points (%d).\e[31m\n", vec_3d.size(), n);
			accuracy = -1.0;
			Eigen::Affine3d a;
			return a;
		}
		
		
		return match_points(
			vec_3d,
			vec_uv,
			fx, fy, cx, cy,
			seed,
			accuracy
		);
	}
	
	Eigen::Affine3d match_points(
		const std::vector<Eigen::Vector2d> vec_3d,
		const std::vector<Eigen::Vector2d> vec_uv,
		const double fx, const double fy, const double cx, const double cy,
		const Eigen::Affine3d & seed,
		double & accuracy
	){
		int n = vec_3d.size();
	
		Eigen::Vector3d ea = seed.rotation().eulerAngles(2, 1, 0);
   	
		double t_array [3] = {
			seed.translation().x(),
			seed.translation().y(),
			seed.translation().z()
		};
		double r_array [3] = {
			cc_utils::rtod(ea.z()),
			cc_utils::rtod(ea.y()),
			cc_utils::rtod(ea.x())
		};
		
		/*n=8;
		std::vector<Eigen::Vector2d> tmp_uv;
		std::vector<Eigen::Vector3d> tmp_3d;
		for(int i = 0; i < n; i++){
			tmp_uv.push_back(vec_uv[i]);
			tmp_3d.push_back(vec_3d[i]);
		}
		vec_uv = tmp_uv;
		vec_3d = tmp_3d;*/
		
		
		//Build the optimization problem
		ceres::Problem problem;
		ceres::Solver::Options options;
		
		cc_utils::init_visualization(2688, 1520, vec_uv, options);
		
		
		
		for(int i = 0; i < n; i++){
			double pixels_as_array [2] = { vec_uv[i].x(), vec_uv[i].y() };
			double target_as_array [3] = { vec_3d[i].x(), vec_3d[i].y(), 0.0 };
		
			ceres::CostFunction *cost_function = CalibrationEntry::Create(
				pixels_as_array,
				target_as_array,
				fx, fy, cx, cy
			);
		
		
			problem.AddResidualBlock(cost_function, NULL,
				t_array, r_array
			);
		}
		cc_utils::bound_rotation(problem, r_array);
		
		#ifdef VERBOSE
		options.minimizer_progress_to_stdout = true;
		#endif
		options.linear_solver_type = ceres::DENSE_SCHUR;
		options.max_num_iterations = 1000;
		ceres::Solver::Summary summary;
    		ceres::Solve(options, &problem, &summary);
	
		Eigen::Affine3d a;
		a =
			Eigen::AngleAxisd(cc_utils::dtor(r_array[2]), Eigen::Vector3d::UnitZ()) *
   			Eigen::AngleAxisd(cc_utils::dtor(r_array[1]), Eigen::Vector3d::UnitY()) *
   			Eigen::AngleAxisd(cc_utils::dtor(r_array[0]), Eigen::Vector3d::UnitX());
   		a.translation() = Eigen::Vector3d(t_array[0], t_array[1], t_array[2]);
   		
   		accuracy = cc_utils::rms();
		return a;
	}
}
