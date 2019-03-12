#pragma once
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "def.h"

class myCalibration
{


public:

	myCalibration();
	~myCalibration();


	/*
	@param infile 目录文件的路径,其中保存了所有用于相机标定的文件名
	@param K      储存相机内部矩阵
	*/
	static void getIntrinsicMat(std::string infile, cv::Mat& K, cv::Mat& distCoeffs);

	static void calcBoardCornerPositions(cv::Size boardSize, float squareSize,
		std::vector<cv::Point3f>& corners
	);

	static void computeReprojectionErrors(
		int imgCount,
		std::vector<std::vector<cv::Point3f>> objectPoints,
		const std::vector<cv::Mat>& rvecsMat,
		const std::vector<cv::Mat>& tvecsMat,
		const cv::Mat& K,
		const cv::Mat& distCoeffs,
		const std::vector<std::vector<cv::Point2f>>& imagePoints,
		std::vector<float>& reprojErrs,
		float* totalErrs
	);


	static void saveToFile(
		std::string outputFileName, 
		cv::Size imageSize, cv::Size boardSize, float squareSize,
		cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
		const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
		const std::vector<float>& reprojErrs,
		const std::vector<std::vector<cv::Point2f> >& imagePoints,
		float* totalAvgErr
	);
};

