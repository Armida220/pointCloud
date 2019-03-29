#pragma once

#include "extracter.h"

namespace feature
{
	class SiftExrtacter : public Extracter
	{
	public:

		SiftExrtacter() 
		{
			setDetector(cv::xfeatures2d::SIFT::create());
		}


		~SiftExrtacter() {}


		bool extractFeatures(
			const cv::Mat& image,
			std::vector<cv::KeyPoint>& keyPoints,
			std::vector<cv::Vec3b>& colors,
			cv::Mat& descriptor
		)
		{
			if (image.empty())
			{
				std::cout << "image is empty" << std::endl;
				return false;
			}

			keyPoints.clear();
			colors.clear();

			// 偶尔出现内存分配失败的错误  Detects keypoints and computes the descriptors
			// sift->detectAndCompute(image, noArray(), key_points, descriptor);
			getDetector()->detect(image, keyPoints);
			getDetector()->compute(image, keyPoints, descriptor);

			// 特征点过少，则排除该图像S
			if (keyPoints.size() <= 10)
				return false;

			// push color into colors
			for (cv::KeyPoint kp : keyPoints)
			{
				cv::Point2f p = kp.pt;
				cv::Vec3b color = image.at<cv::Vec3b>(p.x, p.y);
				colors.push_back(color);
			}

			return true;
		}
	};
}