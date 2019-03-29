#pragma once

//#include <io.h>
#include <iostream>
#include <opencv2/features2d.hpp>

namespace feature
{

	class Extracter
	{
	public:

		Extracter() {}
		
		~Extracter() {}

		/**
		 * @brief this function 提取特征
		 * @param[in] imageVec
		 * @param[in] detector
		 * @param[out] keyPointsVec
		 * @param[out] colorsVec
		 * @param[out] descriptorVec
		 */
		bool extractFeaturesForVector(
			const std::vector<cv::Mat>& imageVec,
			std::vector<std::vector<cv::KeyPoint>>& keyPointsVec,
			std::vector<std::vector<cv::Vec3b>>& colorsVec,
			std::vector<cv::Mat>& descriptorVec
		)
		{
			for (const cv::Mat& image : imageVec)
			{
				std::vector<cv::KeyPoint> keyPoints;
				std::vector<cv::Vec3b> colors;
				cv::Mat descriptor;
				if (extractFeatures(image, keyPoints, colors, descriptor))
				{
					keyPointsVec.push_back(keyPoints);
					colorsVec.push_back(colors);
					descriptorVec.push_back(descriptor);
				}
				else
				{
					return false;
				}
			}
			return true;
		}

		/**
		 * @brief this function 提取特征
		 * @param[in] image
		 * @param[in] detector
		 * @param[out] keyPoints
		 * @param[out] colors
		 * @param[out] descriptor
		 */
		virtual bool extractFeatures(
			const cv::Mat& image,
			std::vector<cv::KeyPoint>& keyPoints,
			std::vector<cv::Vec3b>& colors,
			cv::Mat& descriptor
		) = 0;


		void setDetector(const cv::Ptr<cv::Feature2D>& detector)
		{
			this->detector = detector;
		}

		cv::Ptr<cv::Feature2D> getDetector()
		{
			return this->detector;
		}



	private:
	
		cv::Ptr<cv::Feature2D> detector;

	};


}