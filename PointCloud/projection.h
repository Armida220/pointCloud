#pragma once

#include <opencv2/core/core.hpp>


namespace multiView
{


	/**
	 * @brief this function Compute P*[X|1.0] for the list of point (3D point). 计算投影
	 */
	void project(
		
	);

	/**
	 * @brief this function calculate the positon of points  计算三角投影的三维坐标
	 * @param[in] K
	 * @param[in] R1
	 * @param[in] T1
	 * @param[in] R2
	 * @param[in] T2
	 * @param[in] p1
	 * @param[in] p2
	 * @param[out] structure
	 */
	void triangulateViews(
		const cv::Mat& K,
		const cv::Mat& R1,
		const cv:: Mat& T1,
		const cv::Mat& R2,
		const cv::Mat& T2,
		const std::vector<cv::Point2f>& p1,
		const std::vector<cv::Point2f>& p2,
		//std::vector<cv::Point3f>& structure
		std::vector<cv::Point3d>& structure
	);

	/**
	 * @brief this function Estimates the root mean square error (2D) 计算重投影的标准差
	 */
	double reprojectionErrorRMSE(
	
	);

}