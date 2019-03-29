#include "def.h"

#include "patternDetect.h"
#include "calibration.h"
#include "saveResult.h"
#include "evaluateImages.h"

#include "myCalibration.h"
#include "settings.h"

namespace test
{
	void calibration_test_0()
	{

		std::cout << "calibration test start\n";

		// read settings file
		std::cout << "reading settings file" << std::endl;

		calibration::Settings s;

		const std::string inputSettingsFile = INPUT_SETTINGS_FILE;
		cv::FileStorage fs(inputSettingsFile, cv::FileStorage::READ);
		if (!fs.isOpened())
		{
			std::cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << std::endl;
			return;
		}
		s.read(fs["Settings"]);

		fs.release();

		if (!s.goodInput)
		{
			std::cout << "Invalid input detected. Application stopping. " << std::endl;
			return;
		}

		std::cout << "finish read\n" << std::endl;
		//***********

		std::vector<std::vector<cv::Point3f>> objectPoints; // ����ǵ����ά����
		std::vector<std::vector<cv::Point2f>> imagePoints;  // ����ǵ�Ķ�ά����

		std::vector<cv::Mat> rvecs;  // �洢����ͼ�����ת����
		std::vector<cv::Mat> tvecs;  // �洢����ͼ���ƽ������

		cv::Mat K, distCoeffs;
		std::vector<float> perViewErrs;
		float totalAvgErr = 0;




		// ----------------------��ȡ�ǵ�----------------------------	

		/*
		std::string filename; // ����ͼ����
		std::string filePosition;
		while (getline(fin, filename))
		{
			cv::Mat image = cv::imread(CALIBRATION_IMG_DIR + filename);
			cv::Mat viewGray;
			cv::cvtColor(image, viewGray, cv::COLOR_BGR2GRAY);
			std::vector<cv::Point2f> pointBuf;

			calibration::findPattern(viewGray, s.boardSize, pointBuf);

			imagePoints.push_back(pointBuf);
		}*/

		std::cout << "finding corners from image list" << std::endl;

		for (std::string imageName : s.imageList)
		{
			cv::Mat image = cv::imread(s.calibImgDir + imageName);
			cv::Mat viewGray;
			cv::cvtColor(image, viewGray, cv::COLOR_BGR2GRAY);
			std::vector<cv::Point2f> pointBuf;

			std::cout << imageName << std::endl;
			if (!calibration::findPattern(viewGray, s.boardSize, pointBuf))
			{
				std::cout << "gg" << std::endl;
			}

			imagePoints.push_back(pointBuf);
		}

		std::cout << "corners has found" << std::endl;

		// ----------------------------------------------------------

		// ----------------------����궨----------------------------	

			// ��ʼ���궨���Ͻǵ������
		calibration::computeObjectPoints(s.boardSize, s.squareSize, imagePoints, objectPoints);



		bool calibSuccessed = false;
		// ��calibrateCamera()����������ڲ�����ÿ��ͼƬ�����ԭ���ƽ�ƾ������ת����
		//calibration::runCaliration(objectPoints, imagePoints, imageSize, rvecs, tvecs, K, distCoeffs, perViewErrs, totalAvgErr);


		double maxTotalAvgErr = 1.5;
		int minInputFrames = 8;
		std::vector<float> imageScores;
		std::size_t calibGridSize = 10;
		std::vector<std::vector<std::size_t>> cellIdxPerImage;
		std::vector<std::size_t> cellsWeight;

		calibration::computeCellIndexs(imagePoints, s.imageSize, calibGridSize, cellIdxPerImage);
		calibration::computeCellsWeight(cellIdxPerImage, calibGridSize, cellsWeight);
		calibration::computeImageScores(cellIdxPerImage, cellsWeight, imageScores);
		//for(int i =0; i< cellsWeight.size(); ++i)
		//  std::cout << cellsWeight[i] << std::endl;


		calibSuccessed = calibration::calibrationIterativeOptimization(objectPoints, imagePoints, imageScores,
			s.imageSize, maxTotalAvgErr, minInputFrames,
			rvecs, tvecs, K, distCoeffs, perViewErrs, totalAvgErr);


		// --------------------------------------------------	

		// ----------------------���浽�ļ�----------------------------

		calibration::saveCameraParams(
			s.outputFileName,
			s.imageSize,
			s.boardSize,
			s.squareSize,
			K,
			distCoeffs,
			rvecs,
			tvecs,
			perViewErrs,
			imagePoints,
			totalAvgErr);

	}



	void calibration_test_1()
	{

		// --------------------------- ����궨ģ�� -----------------
			// ��������ڲ�����;�ͷ�������
		cv::Mat K = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
		cv::Mat distCoeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0));
		myCalibration::getIntrinsicMat(CALIBRATION_CATALOG_FILE, K, distCoeffs);
	}




}
