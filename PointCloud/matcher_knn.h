#pragma once

#include "matcher.h"

namespace feature
{
	class KnnMatcher : public Matcher
	{
		void matchFeatures(
			const cv::Mat& queryDescripters,
			const cv::Mat& trainDescripters,
			std::vector<cv::DMatch>& matches
		)
		{
			std::vector<std::vector<cv::DMatch>> knn_matches;
			cv::BFMatcher matcher(cv::NORM_L2);
			matcher.knnMatch(queryDescripters, trainDescripters, knn_matches, 2);

			// 获取满足Ratio Test的最小匹配的距离
			float min_dist = FLT_MAX;
			for (int r = 0; r < knn_matches.size(); ++r)
			{
				// Rotio Test
				if (knn_matches[r][0].distance > 0.6 * knn_matches[r][1].distance)
				{
					continue;
				}

				float dist = knn_matches[r][0].distance;
				if (dist < min_dist)
				{
					min_dist = dist;
				}
			}

			matches.clear();
			for (size_t r = 0; r < knn_matches.size(); ++r)
			{
				// 排除不满足Ratio Test的点和匹配距离过大的点
				if (
					knn_matches[r][0].distance > 0.6 * knn_matches[r][1].distance ||
					knn_matches[r][0].distance > 5 * std::max(min_dist, 10.0f)
					)
				{
					continue;
				}

				// 保存匹配点
				matches.push_back(knn_matches[r][0]);
			}
		}
	};
}