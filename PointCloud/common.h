#pragma once
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace multiView
{


	/**
	 * @brief this function 依据matches从keyPoints数组中提取points
	 * @param[in] kpVec1
	 * @param[in] kpVec2
	 * @param[in] matches
	 * @param[out] pVec1
	 * @param[out] pVec2
	 */
	void getMatchedPoints(
		const std::vector<cv::KeyPoint>& kpVec1, 
		const std::vector<cv::KeyPoint>& kpVec2,
		const std::vector<cv::DMatch>& matches,
		std::vector<cv::Point2f>& pVec1,
		std::vector<cv::Point2f>& pVec2
	);


	/**
	 * @brief this function 依据matches从keyPoints数组中提取points
	 * @param[in] kpVec1
	 * @param[in] kpVec2
	 * @param[in] matches
	 * @param[out] pVec1
	 * @param[out] pVec2
	 */
	void getMatchedColors(
		const std::vector<cv::Vec3b>& cVec1,
		const std::vector<cv::Vec3b>& cVec2,
		const std::vector<cv::DMatch>& matches,
		std::vector<cv::Vec3b>& cVec3,
		std::vector<cv::Vec3b>& cVec4
	);




	/**
	 * @brief this function 依据 mask 过滤掉那些 outline points
	 * @param[in|out] pVec
	 * @param[in] mask
	 */
	void maskoutPoints(std::vector<cv::Point2f>&pVec, const cv::Mat& mask);



	/**
	 * @brief this function 依据 mask 过滤掉那些 outline points
	 * @param[in|out] pVec
	 * @param[in] mask
	 */
	void maskoutColors(std::vector<cv::Vec3b>& cVec, const cv::Mat& mask);

}
