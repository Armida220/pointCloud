#pragma once


#include <vector>
#include <iostream>

#include <opencv2/opencv.hpp>

namespace calibration
{

	bool findPattern(
		const cv::Mat& viewGray, 
		const cv::Size& boardSize, 
		std::vector<cv::Point2f>& pointbuf);



	void calcChessboardCorners( 
		const cv::Size& boardSize,
		const float squareSize,
		std::vector<cv::Point3f>& corners);


	void computeObjectPoints(
		const cv::Size& boardSize, 
		const float squareSize,
		const std::vector<std::vector<cv::Point2f>>& imagePoints,
		std::vector<std::vector<cv::Point3f>>& objectPoints);
}