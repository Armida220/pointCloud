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
	static void getIntrinsicMat(std::string infile, cv::Mat& K);
};

