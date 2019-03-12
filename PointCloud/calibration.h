#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

namespace calibration
{
	/**
	 * @brief This function calibrates the camera. 
	 *
	 */
	bool runCaliration(
		const std::vector<std::vector<cv::Point3f>>& objectPoints,
		const std::vector<std::vector<cv::Point2f>>& imagePoints,
		const cv::Size& imageSize,
		std::vector<cv::Mat> rvecs,
		std::vector<cv::Mat> tvecs,
		cv::Mat& K, 
		cv::Mat& distCoeffs,
		std::vector<float>& perViewErrs,
		float& totalAvgErr);

	/**
	 * @brief This function is the refinement loop of the calibration.
     *
     */
	bool calibrationIterativeOptimization(
		std::vector<std::vector<cv::Point3f>>& calibObjectPoints,
		std::vector<std::vector<cv::Point2f>>& calibImagePoints,
		std::vector<float> calibImageScores,
		const cv::Size& imageSize,
		const double& maxTotalAvgErr,
		const int& minInputFrames,
		std::vector<cv::Mat> rvecs,
		std::vector<cv::Mat> tvecs,
		cv::Mat& K,
		cv::Mat& distCoeffs,
		std::vector<float>& perViewErrs,
		float& totalAvgErr);
	
	/**
	 * @brief This function computes the average of the reprojection errors.
	 *
	 */
	float computeReprojectionErrors(
		const std::vector<std::vector<cv::Point3f>>& objectPoints,
		const std::vector<std::vector<cv::Point2f>>& imagePoints,
		const std::vector<cv::Mat>& rvecs,
		const std::vector<cv::Mat>& tvecs,
		const cv::Mat& K,
		const cv::Mat& distCoeffs,
		std::vector<float>& perViewErrs);

}