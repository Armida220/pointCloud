#pragma once

#include <opencv2/features2d.hpp>

namespace feature
{
	/**
	 * @brief this class is a virtual class
     */
	class Matcher
	{

	public:

		Matcher() {}

		~Matcher() {}

		/**
		 * @brief this function match features descripters from a set of ordered images
		 * @param[in] descriptorVec a vector which store every view's descriptors.
		 * @param[out] matchesVec a vector to store match between each view and it's next view. 
		 */
		void matchFeaturesForVector(
			const std::vector<cv::Mat>& descriptorVec,
			std::vector<std::vector<cv::DMatch>>& matchesVec
		) 
		{
			matchesVec.clear();
			for (int i = 0; i < descriptorVec.size() - 1; ++i)
			{
				std::vector<cv::DMatch> matches;
				matchFeatures(descriptorVec[i], descriptorVec[i + 1], matches);
				matchesVec.push_back(matches);
			}
		}

		/**
		 * @brief this function match features descripters from two images
		 * @param[in] queryDescripters
		 * @param[in] trainDescripters
		 * @param[out] matches
		 */
		virtual void matchFeatures(
			const cv::Mat& queryDescripters,
			const cv::Mat& trainDescripters,
			std::vector<cv::DMatch>& matches
		) = 0;
	};

}