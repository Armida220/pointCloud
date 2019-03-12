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
		// ��ʼ�����
		rvecs.resize(0);
		tvecs.resize(0);
		perViewErrs.resize(0);
		K = cv::Mat::eye(3, 3, CV_64F);
		distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
		totalAvgErr = 0;

		// ��calibrateCamera()����������ڲ�����ÿ��ͼƬ�����ԭ���ƽ�ƾ������ת����
		cv::calibrateCamera(objectPoints, imagePoints, imageSize, K, distCoeffs, rvecs, tvecs);
		//std::cout << K;

		// ������ͶӰ���
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
		// ��ʼ������
		int calibIteration = 0;
		bool calibSucceeded = false;
		totalAvgErr = 0;

		// �����Ż�
		do
		{
			//����һ�α궨
			calibSucceeded = runCaliration(
				calibObjectPoints, calibImagePoints, imageSize, rvecs, tvecs,
				K, distCoeffs, perViewErrs, totalAvgErr);

			// �궨����Ҫ���˳�
			if (totalAvgErr <= maxTotalAvgErr)
			{
				break;
			}
			// �궨������ͼƬ̫�٣��˳�
			else if (calibObjectPoints.size() < minInputFrames)
			{
				break;
			}
			// �궨������Ҫ��
			else if(calibSucceeded)
			{
				// ��������ͼƬ�����
				std::vector<float> globalScores;
				for (int i = 0; i < calibObjectPoints.size(); ++i)
				{
					globalScores.push_back(perViewErrs[i] * calibImageScores[i]);
				}

				// ��ȡ���������С���
				const auto minMaxErr = std::minmax_element(globalScores.begin(), globalScores.end());
				
				// ������ͼƬ��һ�����޷��޳�ͼƬ��ֱ�ӱ궨ʧ�ܲ��˳�
				if (*minMaxErr.first == *minMaxErr.second)
				{
					std::cout << "Same error on all images:" << *minMaxErr.first << std::endl;
					break;
				}

				// �޳���Щ���ϴ��ͼƬ
				const float errThreshold = *minMaxErr.first + 0.8f * (*minMaxErr.second - *minMaxErr.first);
				
				// �������Ҫ���ͼƬ
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

				// ���ϵ�ͼƬ̫�٣��˳�
				if (filteredObjectPoints.size() < minInputFrames)
				{
					std::cout << "Not enough filtered input images" << std::endl;
					break;
				}

				// ����ͼƬ�����ϣ��㷨�������˳�
				if (filteredObjectPoints.size() == calibObjectPoints.size())
				{
					std::cout << "Convergence reached" << std::endl;
					break;
				}

				// ��ɸѡ��ͼƬ��Ϊ���룬�ٴε���
				calibImagePoints.swap(filteredImagePoints);
				calibObjectPoints.swap(filteredObjectPoints);
				calibImageScores.swap(filteredImageScores);
			}

			// ��������+1
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
		// ��ʼ��
		int totalPoints = 0;
		float err = 0.0;
		float totalErr = 0;
		perViewErrs.resize(objectPoints.size());

		// ����ÿ��ͼƬ����ͶӰ���
		for (int i = 0; i < objectPoints.size(); i++)
		{
			std::vector<cv::Point2f> imagePoints2;

			// ͨ�������������������̵Ľǵ��������ͶӰ���㣬���浽image_points2��
			cv::projectPoints(objectPoints[i], rvecs[i], tvecs[i], K, distCoeffs, imagePoints2);

			// ������ͶӰ���ͶӰ��֮������
			err = cv::norm(imagePoints[i], imagePoints2, cv::NORM_L2);

			// �����ͼƬ��ͶӰ���ƽ�����
			int n = objectPoints[i].size();
			perViewErrs[i] = (float)std::sqrt(err*err / n);

			totalErr += err * err;
			totalPoints += n;
		}

		// ��������ͼƬ��ͶӰ���ƽ�����
		totalErr = std::sqrt(totalErr / totalPoints);
		return totalErr;
	}
}
