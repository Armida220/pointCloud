#pragma once

#include <opencv2/core/core.hpp>

namespace multiView
{

	/**
	 * @brief this function Estimate an E-matrix from a given set of point matches
	 * @param[in] K camera matrix
	 * @param[in] p1 
	 * @param[in] p2 
	 * @param[out] E the essential matrix
	 * @param[out] mask 
	 */
	int estimateEMatrix(
		const cv::Mat& K,
		const std::vector<cv::Point2f>& p1,
		const std::vector<cv::Point2f>& p2,
		cv::Mat& E,
		cv::Mat& mask
	);


	int estimateFMatrix(
		const std::vector<cv::Point2f>& p1,
		const std::vector<cv::Point2f>& p2,
		cv::Mat& F,
		cv::Mat& mask
	);


	/**
	 * @brief this function 计算相机的外参
	 * @param[in] K
	 * @param[in] p1
	 * @param[in] p2
	 * @param[out] R
	 * @param[out] T
	 * @param[out] mask
	 */
	bool estimatePose(
		const cv::Mat& K, 
		const std::vector<cv::Point2f>& p1,
		const std::vector<cv::Point2f>& p2,
		cv::Mat& R,
		cv::Mat& T,
		cv::Mat& mask
	);

}

