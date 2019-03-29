#include "sfm.h"

namespace sfm
{
	SfM::SfM() {}

	SfM::~SfM() {}

	void SfM::runSfM()
	{	

		// input images
		if (this->consoleDebugLevel <= LOG_DEBUG)
			std::cout << "is reading images from the given dir" << std::endl;
		imageIO::readImage(this->imageDir, this->imageVec);
		if (this->consoleDebugLevel <= LOG_DEBUG)
			std::cout << "---------------DONE----------------" << std::endl;


		// extract features
		this->detector = cv::xfeatures2d::SIFT::create();
		
		if (this->consoleDebugLevel <= LOG_DEBUG)
			std::cout << "is extracting features from all input images" << std::endl;
		
		extractFeatures(this->extracterType);
		
		if (this->consoleDebugLevel <= LOG_DEBUG)
			std::cout << "---------------DONE----------------" << std::endl;


		// match features
		if (this->consoleDebugLevel <= LOG_DEBUG)
			std::cout << "is matching features" << std::endl;

		matchFeatures(this->matcherType);
		
		if (this->consoleDebugLevel <= LOG_DEBUG)
			std::cout << "---------------DONE----------------" << std::endl;


		// init the first two struct
		// add more struct into pointcloud
		multiView::generalRebuild(K, distCoeffs,
			keyPointsVec,colorsVec, matchesVec,
			structure, correspond_struct_idx, colors,
			rotations, motions
		);


		// BA
		cv::Mat intrinsic(cv::Matx41d(K.at<double>(0, 0), K.at<double>(1, 1), K.at<double>(0, 2), K.at<double>(1, 2)));
		std::vector<cv::Mat> extrinsics;
		for (size_t i = 0; i < rotations.size(); ++i)
		{
			cv::Mat extrinsic(6, 1, CV_64FC1);
			cv::Mat r;
			Rodrigues(rotations[i], r);

			r.copyTo(extrinsic.rowRange(0, 3));
			motions[i].copyTo(extrinsic.rowRange(3, 6));

			extrinsics.push_back(extrinsic);
		}

		bundleAdjustment::bundleAdjustment(intrinsic, extrinsics, correspond_struct_idx, keyPointsVec, structure);


		// save the rebuild result to file
		if (this->consoleDebugLevel <= LOG_DEBUG)
			std::cout << "start saving struct result to " << this->saveFileName << std::endl;
		sfm::save2Yml(this->saveFileName, this->rotations, this->motions, this->structure, this->colors);
		//sfm::save2Ply("structure.ply", this->structure, this->colors);
		if (this->consoleDebugLevel <= LOG_DEBUG)
			std::cout << "---------------DONE----------------" << std::endl;

	}

	void SfM::initSfM(std::string configFile)
	{
		if (this->consoleDebugLevel <= LOG_INFO)
			std::cout << "start reading config file" << std::endl;

		cv::FileStorage fs(configFile, cv::FileStorage::READ);
		if (!fs.isOpened())
		{
			if (this->consoleDebugLevel <= LOG_INFO)
			    std::cout << "Could not open the configuration file: \"" << configFile << "\"" << std::endl;
			return;
		}
		bool successed = readConfigFile(fs["Settings"]);
		fs.release();

		if (!successed)
		{
			if (this->consoleDebugLevel <= LOG_INFO)
			    std::cout << "There are some illegal settings in your config files." << std::endl;
			exit(0);
		}

		if (this->consoleDebugLevel <= LOG_INFO)
			std::cout << "---------------DONE----------------" << std::endl;
	}


	bool SfM::readConfigFile(const cv::FileNode& node)
	{

		setConsoleDebugLevel(DebugLogLevel::LOG_DEBUG);

		node["Image_Dir"] >> this->imageDir;
		node["Save_File"] >> this->saveFileName;

		// if [calibrated] equals 0, use calibration mode, else use [camera_matrix]
		int calibrated = 0;
		node["Calibrated"] >> calibrated;
		if (calibrated > 0)
		{
			node["camera_matrix"] >> this->K;
			node["distortion_coefficients"] >> this->distCoeffs;
		}
		else
		{
			std::string calibConfigFile;
			node["Calibration_Config_File"] >> calibConfigFile;
			bool calibSuccessed = calibrate(calibConfigFile, this->K, this->distCoeffs);
			if (!calibSuccessed) return false;
		}


		// set extracter_type
		std::string extracterType;
		node["Extracter_Type"] >> extracterType;
		this->extracterType = feature::stringToExtracterType(extracterType);
		
		// set matcher_type
		std::string matcherType;
		node["Matcher_Type"] >> matcherType;
		this->matcherType = feature::stringToMatcherType(matcherType);


		return validateConfig();
	}

	bool SfM::validateConfig()
	{
		if (this->K.empty()) return false;
		if (this->distCoeffs.empty()) return false;


		return true;
	}


	bool SfM::calibrate(const std::string& calibConfigFile, cv::Mat& K, cv::Mat& distCoeffs)
	{

		std::cout << "calibration test start\n";

		// read settings file
		std::cout << "reading settings file" << std::endl;

		calibration::Settings s;

		cv::FileStorage fs(calibConfigFile, cv::FileStorage::READ);
		if (!fs.isOpened())
		{
			std::cout << "Could not open the configuration file: \"" << calibConfigFile << "\"" << std::endl;
			return false;
		}
		s.read(fs["Settings"]);

		fs.release();

		if (!s.goodInput)
		{
			std::cout << "Invalid input detected. Application stopping. " << std::endl;
			return false;
		}

		std::cout << "finish read\n" << std::endl;
		//***********

		std::vector<std::vector<cv::Point3f>> objectPoints; // 保存角点的三维坐标
		std::vector<std::vector<cv::Point2f>> imagePoints;  // 保存角点的二维坐标

		std::vector<cv::Mat> rvecs;  // 存储所有图像的旋转向量
		std::vector<cv::Mat> tvecs;  // 存储所有图像的平移向量


		std::vector<float> perViewErrs;
		float totalAvgErr = 0;


		// ----------------------提取角点----------------------------	

		/*
		std::string filename; // 缓存图像名
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

		// ----------------------相机标定----------------------------	

			// 初始化标定板上角点的坐标
		calibration::computeObjectPoints(s.boardSize, s.squareSize, imagePoints, objectPoints);

		bool calibSuccessed = false;
		// 用calibrateCamera()计算相机的内部矩阵、每张图片相对于原点的平移矩阵和旋转矩阵
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


		// save calibration result to file
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

		return true;
	}



	void SfM::extractFeatures(const feature::ExtracterType& extracterType)
	{
		feature::Extracter* extracter;

		if (extracterType == feature::ExtracterType::SIFT)
		{
			std::cout << "use SIFT extracter" << std::endl;
			extracter = new feature::SiftExrtacter;
		}
		else
		{
			std::cout << "use SURF extracter" << std::endl;
			extracter = new feature::SiftExrtacter;
		}

		extracter->extractFeaturesForVector(this->imageVec, this->keyPointsVec, this->colorsVec, this->descriptorVec);

	}



	void SfM::matchFeatures(const feature::MatcherType& matcherType)
	{
		feature::Matcher* matcher;

		if (matcherType == feature::MatcherType::Knn)
		{
			std::cout << "use Knn matcher" << std::endl;
			matcher = new feature::KnnMatcher;
		}
		else
		{
			std::cout << "use BF matcher" << std::endl;
			matcher = new feature::BFMatcher;
		}

		matcher->matchFeaturesForVector(this->descriptorVec, this->matchesVec);
	}

}