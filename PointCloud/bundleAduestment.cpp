#include "bundleAdjustment.h"

namespace bundleAdjustment
{
	void bundleAdjustment(
		cv::Mat& intrinsic,
		std::vector<cv::Mat>& extrinsics,
		std::vector<std::vector<int>>& correspond_struct_idx,
		std::vector<std::vector<cv::KeyPoint>>& key_points_for_all,
		std::vector<cv::Point3d>& structure
	)
	{
		ceres::Problem problem;

		// load extrinsics (rotations and motions)
		for (size_t i = 0; i < extrinsics.size(); ++i)
		{
			problem.AddParameterBlock(extrinsics[i].ptr<double>(), 6);
		}
		// fix the first camera.
		problem.SetParameterBlockConstant(extrinsics[0].ptr<double>());

		// load intrinsic
		problem.AddParameterBlock(intrinsic.ptr<double>(), 4); // fx, fy, cx, cy

		// load points
		ceres::LossFunction* loss_function = new ceres::HuberLoss(4);   // loss function make bundle adjustment robuster.
		for (size_t img_idx = 0; img_idx < correspond_struct_idx.size(); ++img_idx)
		{
			std::vector<int>& point3d_ids = correspond_struct_idx[img_idx];
			std::vector<cv::KeyPoint>& key_points = key_points_for_all[img_idx];
			for (size_t point_idx = 0; point_idx < point3d_ids.size(); ++point_idx)
			{
				int point3d_id = point3d_ids[point_idx];
				if (point3d_id < 0)
					continue;

				cv::Point2d observed = key_points[point_idx].pt;
				// 模板参数中，第一个为代价函数的类型，第二个为代价的维度，剩下三个分别为代价函数第一第二还有第三个参数的维度
				ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<ReprojectCost, 2, 4, 6, 3>(new ReprojectCost(observed));

				problem.AddResidualBlock(
					cost_function,
					loss_function,
					intrinsic.ptr<double>(),            // Intrinsic
					extrinsics[img_idx].ptr<double>(),  // View Rotation and Translation
					&(structure[point3d_id].x)          // Point in 3D space
				);
			}
		}

		// Solve BA
		ceres::Solver::Options ceres_config_options;
		ceres_config_options.minimizer_progress_to_stdout = false;
		ceres_config_options.logging_type = ceres::SILENT;
		ceres_config_options.num_threads = 1;
		ceres_config_options.preconditioner_type = ceres::JACOBI;
		ceres_config_options.linear_solver_type = ceres::SPARSE_SCHUR;
		ceres_config_options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;

		ceres::Solver::Summary summary;
		ceres::Solve(ceres_config_options, &problem, &summary);

		if (!summary.IsSolutionUsable())
		{
			std::cout << "Bundle Adjustment failed." << std::endl;
		}
		else
		{
			// Display statistics about the minimization
			std::cout << std::endl
				<< "Bundle Adjustment statistics (approximated RMSE):\n"
				<< " #views: " << extrinsics.size() << "\n"
				<< " #residuals: " << summary.num_residuals << "\n"
				<< " Initial RMSE: " << std::sqrt(summary.initial_cost / summary.num_residuals) << "\n"
				<< " Final RMSE: " << std::sqrt(summary.final_cost / summary.num_residuals) << "\n"
				<< " Time (s): " << summary.total_time_in_seconds << "\n"
				<< std::endl;
		}
	}
}
