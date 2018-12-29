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
	@param infile Ŀ¼�ļ���·��,���б�����������������궨���ļ���
	@param K      ��������ڲ�����
	*/
	static void getIntrinsicMat(std::string infile, cv::Mat& K);
};

