#include <opencv2/calib3d.hpp>

#include <ctime>
#include <algorithm>
#include <iostream>

#include "calibration.h"


namespace calibration
{


	bool runCaliration(
		const std::vector<std::vector<cv::Point3f>>& objectPoints,
		const std::vector<std::vector<cv::Point2f>>& imagePoints,
		const cv::Size& imageSize,
		std::vector<cv::Mat> rvecs,
		std::vector<cv::Mat> tvecs,
		cv::Mat& K,
		cv::Mat& distCoeffs,
		std::vector<float>& perViewErrs,
		float& totalAvgErr)
	{	
		// 初始化输出
		rvecs.resize(0);
		tvecs.resize(0);
		perViewErrs.resize(0);
		K = cv::Mat::eye(3, 3, CV_64F);
		distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
		totalAvgErr = 0;

		// 用calibrateCamera()计算相机的内部矩阵、每张图片相对于原点的平移矩阵和旋转矩阵
		cv::calibrateCamera(objectPoints, imagePoints, imageSize, K, distCoeffs, rvecs, tvecs);
		//std::cout << K;

		// 计算重投影误差
		totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, K, distCoeffs, perViewErrs);

		bool ok = cv::checkRange(K) && cv::checkRange(distCoeffs);
		return ok;
	}

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
		float& totalAvgErr)
	{
		// 初始化变量
		int calibIteration = 0;
		bool calibSucceeded = false;
		totalAvgErr = 0;

		// 迭代优化
		do
		{
			//进行一次标定
			calibSucceeded = runCaliration(
				calibObjectPoints, calibImagePoints, imageSize, rvecs, tvecs,
				K, distCoeffs, perViewErrs, totalAvgErr);

			// 标定符合要求，退出
			if (totalAvgErr <= maxTotalAvgErr)
			{
				break;
			}
			// 标定的输入图片太少，退出
			else if (calibObjectPoints.size() < minInputFrames)
			{
				break;
			}
			// 标定不符合要求
			else if(calibSucceeded)
			{
				// 计算所有图片的误差
				std::vector<float> globalScores;
				for (int i = 0; i < calibObjectPoints.size(); ++i)
				{
					globalScores.push_back(perViewErrs[i] * calibImageScores[i]);
				}

				// 获取最大误差和最小误差
				const auto minMaxErr = std::minmax_element(globalScores.begin(), globalScores.end());
				
				// 若所有图片误差都一样，无法剔除图片，直接标定失败并退出
				if (*minMaxErr.first == *minMaxErr.second)
				{
					std::cout << "Same error on all images:" << *minMaxErr.first << std::endl;
					break;
				}

				// 剔除那些误差较大的图片
				const float errThreshold = *minMaxErr.first + 0.8f * (*minMaxErr.second - *minMaxErr.first);
				
				// 保存符合要求的图片
				std::vector<std::vector<cv::Point2f>> filteredImagePoints;
				std::vector<std::vector<cv::Point3f>> filteredObjectPoints;
				std::vector<float> filteredImageScores;


				for (int i = 0; i < calibObjectPoints.size(); i++)
				{
					if (globalScores[i] < errThreshold)
					{
						filteredImagePoints.push_back(calibImagePoints[i]);
						filteredObjectPoints.push_back(calibObjectPoints[i]);
						filteredImageScores.push_back(calibImageScores[i]);
					}
				}

				// 符合的图片太少，退出
				if (filteredObjectPoints.size() < minInputFrames)
				{
					std::cout << "Not enough filtered input images" << std::endl;
					break;
				}

				// 所有图片都符合，算法收敛，退出
				if (filteredObjectPoints.size() == calibObjectPoints.size())
				{
					std::cout << "Convergence reached" << std::endl;
					break;
				}

				// 将筛选的图片作为输入，再次迭代
				calibImagePoints.swap(filteredImagePoints);
				calibObjectPoints.swap(filteredObjectPoints);
				calibImageScores.swap(filteredImageScores);
			}

			// 迭代次数+1
			++calibIteration;
			std::cout << "\ncalibIteration:" << calibIteration << calibImagePoints.size() << "\n";
		} while (calibSucceeded);


		std::cout << calibSucceeded ? "calibration succeseded" : "calibration failed";
		return calibSucceeded;
	}

	float computeReprojectionErrors(
		const std::vector<std::vector<cv::Point3f>>& objectPoints,
		const std::vector<std::vector<cv::Point2f>>& imagePoints,
		const std::vector<cv::Mat>& rvecs,
		const std::vector<cv::Mat>& tvecs,
		const cv::Mat& K,
		const cv::Mat& distCoeffs,
		std::vector<float>& perViewErrs) 
	{
		// 初始化
		int totalPoints = 0;
		float err = 0.0;
		float totalErr = 0;
		perViewErrs.resize(objectPoints.size());

		// 计算每幅图片的重投影误差
		for (int i = 0; i < objectPoints.size(); i++)
		{
			std::vector<cv::Point2f> imagePoints2;

			// 通过相机内外参数，对棋盘的角点进行重新投影计算，保存到image_points2中
			cv::projectPoints(objectPoints[i], rvecs[i], tvecs[i], K, distCoeffs, imagePoints2);

			// 计算重投影点和投影点之间的误差
			err = cv::norm(imagePoints[i], imagePoints2, cv::NORM_L2);

			// 计算该图片上投影点的平均误差
			int n = objectPoints[i].size();
			perViewErrs[i] = (float)std::sqrt(err*err / n);

			totalErr += err * err;
			totalPoints += n;
		}

		// 计算所有图片上投影点的平均误差
		totalErr = std::sqrt(totalErr / totalPoints);
		return totalErr;
	}
}
