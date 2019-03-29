#pragma once

#include <opencv2/core/core.hpp>

namespace multiView
{
	/**
	 * @brief this function 始化点云，也就是通过双目重建方法对图像序列的头两幅图像进行重建，并初始化correspond_struct_idx
	 * @param[in] K
	 * @param[in] key_points_for_all
	 * @param[in] matches_for_all
	 * @param[out] structure
	 * @param[out] correspond_struct_idx
	 * @param[out] rotations
	 * @param[out] motions
	 */
	void findBaselineTriangulation(
		const cv::Mat K,
		const std::vector<std::vector<cv::KeyPoint>>& key_points_for_all,
	    const std::vector<std::vector<cv::Vec3b>>& colors_for_all,
		const std::vector<std::vector<cv::DMatch>>& matches_for_all,
		//std::vector<cv::Point3f>& structure,
		std::vector<cv::Point3d>& structure,
		std::vector<std::vector<int>>& correspond_struct_idx,
		std::vector<cv::Vec3b>& colors,
		std::vector<cv::Mat>& rotations,
		std::vector<cv::Mat>& motions
	);

	
	/**
	 * @brief this function 增量化重建
	 * @param[in] K
	 * @param[in] key_points_for_all
	 * @param[in] matches_for_all
	 * @param[out] structure
	 * @param[out] correspond_struct_idx
	 * @param[out] rotations
	 * @param[out] motions
	 */
	void generalRebuild(
		const cv::Mat& K,
		const cv::Mat& distCoeffs,
		const std::vector<std::vector<cv::KeyPoint>>& key_points_for_all,
	    const std::vector<std::vector<cv::Vec3b>>& colors_for_all,
		const std::vector<std::vector<cv::DMatch>>& matches_for_all,
		//std::vector<cv::Point3f>& structure,
		std::vector<cv::Point3d>& structure,
		std::vector<std::vector<int>>& correspond_struct_idx,
		std::vector<cv::Vec3b>& colors,
		std::vector<cv::Mat>& rotations,
		std::vector<cv::Mat>& motions
	);


	/**
	 * @brief this function 空间点坐标 和 对应的像素坐标
	 * @param[in] K
	 * @param[in] key_points_for_all
	 * @param[in] matches
	 * @param[out] structure
	 * @param[out] correspond_struct_idx
	 * @param[out] rotations
	 * @param[out] motions
	 */
	void get_objpoints_and_imgpoints(
		const std::vector<cv::DMatch>& matches,
		const std::vector<int>& struct_indices,
		//const std::vector<cv::Point3f>& structure,
		const std::vector<cv::Point3d>& structure,
		const std::vector<cv::KeyPoint>& key_points,
		std::vector<cv::Point3f>& object_points,
		std::vector<cv::Point2f>& image_points
	);


	/**
	 * @brief this function 融合点云
	 * @param[in] K
	 * @param[in] key_points_for_all
	 * @param[in] matches
	 * @param[out] structure
	 * @param[out] correspond_struct_idx
	 * @param[out] rotations
	 * @param[out] motions
	 */
	void fusion_structure(
		const std::vector<cv::DMatch>& matches,
		std::vector<int>& struct_indices,
		std::vector<int>& next_struct_indices,
		//std::vector<cv::Point3f>& structure,
		std::vector<cv::Point3d>& structure,
		//std::vector<cv::Point3f>& next_structure,
		std::vector<cv::Point3d>& next_structure,
		std::vector<cv::Vec3b>& colors,
		std::vector<cv::Vec3b>& next_colors
	);

}