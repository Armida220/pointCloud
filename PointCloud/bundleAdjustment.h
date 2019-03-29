#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <opencv2/core/core.hpp>



namespace bundleAdjustment
{

	// ���ۺ���
	struct ReprojectCost
	{
		cv::Point2d observation;

		ReprojectCost(cv::Point2d& observation)
			: observation(observation)
		{
		}

		template <typename T>
		bool operator()(const T* const intrinsic, const T* const extrinsic, const T* const pos3d, T* residuals) const
		{
			const T* r = extrinsic;
			const T* t = &extrinsic[3];

			T pos_proj[3];
			ceres::AngleAxisRotatePoint(r, pos3d, pos_proj);


			// Apply the camera translation
			pos_proj[0] += t[0];
			pos_proj[1] += t[1];
			pos_proj[2] += t[2];

			const T x = pos_proj[0] / pos_proj[2];
			const T y = pos_proj[1] / pos_proj[2];

			const T fx = intrinsic[0];
			const T fy = intrinsic[1];
			const T cx = intrinsic[2];
			const T cy = intrinsic[3];

			// Apply intrinsic
			const T u = fx * x + cx;
			const T v = fy * y + cy;

			residuals[0] = u - T(observation.x);
			residuals[1] = v - T(observation.y);

			return true;
		}
	};



	void bundleAdjustment(
		cv::Mat& intrinsic,
		std::vector<cv::Mat>& extrinsics,
		std::vector<std::vector<int>>& correspond_struct_idx,
		std::vector<std::vector<cv::KeyPoint>>& key_points_for_all,
		std::vector<cv::Point3d>& structure
	);

}