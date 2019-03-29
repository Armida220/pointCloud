//#include "extractFeatures.h"
//
//namespace feature
//{
//	bool extractFeaturesForVector(
//		const std::vector<cv::Mat>& imageVec,
//		cv::Ptr<cv::Feature2D>& detector,
//		std::vector<std::vector<cv::KeyPoint>>& keyPointsVec,
//		std::vector<std::vector<cv::Vec3b>>& colorsVec,
//		std::vector<cv::Mat>& descriptorVec
//	)
//	{
//		for (const cv::Mat& image : imageVec)
//		{
//			std::vector<cv::KeyPoint> keyPoints;
//			std::vector<cv::Vec3b> colors;
//			cv::Mat descriptor;
//			if (extractFeatures(image, detector, keyPoints, colors, descriptor))
//			{
//				keyPointsVec.push_back(keyPoints);
//				colorsVec.push_back(colors);
//				descriptorVec.push_back(descriptor);
//			}
//			else
//			{
//				return false;
//			}
//		}
//		return true;
//	}
//
//	bool extractFeatures(
//		const cv::Mat& image,
//		cv::Ptr<cv::Feature2D>& detector,
//		std::vector<cv::KeyPoint>& keyPoints,
//		std::vector<cv::Vec3b>& colors,
//		cv::Mat& descriptor
//	)
//	{
//		if (image.empty())
//		{
//			std::cout << "image is empty" << std::endl;
//			return false;
//		}
//
//		keyPoints.clear();
//		colors.clear();
//
//		// 偶尔出现内存分配失败的错误  Detects keypoints and computes the descriptors
//		// sift->detectAndCompute(image, noArray(), key_points, descriptor);
//		detector->detect(image, keyPoints);
//		detector->compute(image, keyPoints, descriptor);
//
//		// 特征点过少，则排除该图像S
//		if (keyPoints.size() <= 10)
//			return false;
//
//		// push color into colors
//		for (cv::KeyPoint kp : keyPoints)
//		{
//			cv::Point2f p = kp.pt;
//			cv::Vec3b color = image.at<cv::Vec3b>(p.x, p.y);
//			colors.push_back(color);
//		}
//
//		return true;
//	}
//}