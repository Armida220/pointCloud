#pragma once
#include <io.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "image.h"

#include "extracterType.h"
#include "extracter_sift.h"

#include "matcherType.h"
#include "matcher_knn.h"
#include "matcher_bruteForce.h"

#include "stereo.h"

#include "sfmDataIO.h"

#include "bundleAdjustment.h"

#include "patternDetect.h"
#include "calibration.h"
#include "saveResult.h"
#include "evaluateImages.h"
#include "settings.h"

namespace sfm
{
	enum DebugLogLevel
	{
		LOG_TRACE = 0,
		LOG_DEBUG,
		LOG_INFO,
		LOG_WARN,
		LOG_ERROR
	};

	class SfM
	{
	public:
		SfM();

		~SfM();


		/**
		 * @brief this function run sfm
		 */
		void runSfM();

		/**
		 * @brief this function is used to init sfm.
		 * @param[in] configFile is the path of config file for sfm.
		 */
		void initSfM(std::string configFile);


		void setConsoleDebugLevel(int debugLevel)
		{
			this->consoleDebugLevel = MIN(debugLevel, LOG_ERROR);
		}

		void printSfMConfig()
		{
			return;
		}



	private:

		/**
		 * @brief this function is used to read settings from file.
		 * @param[in] node is the path of config file for sfm.
		 */
		bool readConfigFile(const cv::FileNode& node);

		/**
		 * @brief this function is used to validate all settings.
		 * @param[in] configFile is the path of config file for sfm.
		 */
		bool validateConfig();

		/**
		 * @brief this function call calibration mod to calculate params of camera.
		 * @param[in] calibConfigFile is the config file for calibration.
		 * @param[out] K is a 3*3 matrix of the camera matrix.
		 * @param[out] distCoeffs is 5*1 matrix of the camera matrix.
		 */
		bool calibrate(const std::string& calibConfigFile, cv::Mat& K, cv::Mat& distCoeffs);

		/**
		 * @brief this function use certain method to extract keypoints from views.
		 * @param[in] extracterType
		 */
		void extractFeatures(const feature::ExtracterType& extracterType);

		/**
		 * @brief this function use certain method to match discriptors.
		 * @param[in] matcherTper
		 */
		void matchFeatures(const feature::MatcherType& matcherType);

		std::string imageDir;
		std::vector<cv::Mat> imageVec;
		
		// params of camera
		cv::Mat K;
		cv::Mat distCoeffs;

		// temp vectors 
		cv::Ptr<cv::Feature2D> detector;
		std::vector<std::vector<cv::KeyPoint>> keyPointsVec;
		std::vector<std::vector<cv::Vec3b>> colorsVec;
		std::vector<cv::Mat> descriptorVec;

		std::vector<std::vector<cv::DMatch>> matchesVec;

		// vectors for output
		//std::vector<cv::Point3f> structure;
		std::vector<cv::Point3d> structure;
		std::vector<std::vector<int>> correspond_struct_idx;
		std::vector<cv::Mat> rotations;
		std::vector<cv::Mat> motions;
		std::vector<cv::Vec3b> colors;

		std::string saveFileName;

		int consoleDebugLevel;
		feature::MatcherType matcherType;
		feature::ExtracterType extracterType;

	};

}