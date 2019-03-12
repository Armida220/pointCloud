#pragma once
#include <opencv2/opencv.hpp>

#include <vector>
#include <string>

namespace calibration
{
	/**
	 * @brief This function saves some parameters' camera into a txt file.
	 */
	void saveCameraParams(
		const std::string& filename,
		const cv::Size& imageSize, 
		const cv::Size& boardSize,
	    const float& squareSize, 
		const cv::Mat& K,
		const cv::Mat& distCoeffs,
		const std::vector<cv::Mat>& rvecs, 
		const std::vector<cv::Mat>& tvecs,
		const std::vector<float>& reprojErrs,
		const std::vector<std::vector<cv::Point2f> >& imagePoints,
		const double& totalAvgErr);
}