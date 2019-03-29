#include "common.h"

namespace multiView
{
	void getMatchedPoints(
		const std::vector<cv::KeyPoint>& kpVec1,
		const std::vector<cv::KeyPoint>& kpVec2,
		const std::vector<cv::DMatch>& matches,
		std::vector<cv::Point2f>& pVec1,
		std::vector<cv::Point2f>& pVec2
	)
	{
		pVec1.clear();
		pVec2.clear();

		for (cv::DMatch match : matches)
		{
			pVec1.push_back(kpVec1[match.queryIdx].pt);
			pVec2.push_back(kpVec2[match.trainIdx].pt);
		}
	}


	void getMatchedColors(
		const std::vector<cv::Vec3b>& cVec1,
		const std::vector<cv::Vec3b>& cVec2,
		const std::vector<cv::DMatch>& matches,
		std::vector<cv::Vec3b>& cVec3,
		std::vector<cv::Vec3b>& cVec4
	)
	{
		cVec3.clear();
		cVec4.clear();

		for (cv::DMatch match : matches)
		{
			cVec3.push_back(cVec1[match.queryIdx]);
			cVec4.push_back(cVec2[match.trainIdx]);
		}
	}


	void maskoutPoints(std::vector<cv::Point2f>&pVec, const cv::Mat& mask)
	{
		std::vector<cv::Point2f> temp = pVec;
		pVec.clear();

		for (int i = 0; i < mask.rows; ++i)
		{
			if (mask.at<uchar>(i) > 0)
				pVec.push_back(temp[i]);
		}
	}

	void maskoutColors(std::vector<cv::Vec3b>& cVec, const cv::Mat& mask)
	{
		std::vector<cv::Vec3b> temp = cVec;
		cVec.clear();

		for (int i = 0; i < mask.rows; ++i)
		{
			if (mask.at<uchar>(i) > 0)
				cVec.push_back(temp[i]);
		}
	}

}