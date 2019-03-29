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

			// ��ȡ����Ratio Test����Сƥ��ľ���
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
				// �ų�������Ratio Test�ĵ��ƥ��������ĵ�
				if (
					knn_matches[r][0].distance > 0.6 * knn_matches[r][1].distance ||
					knn_matches[r][0].distance > 5 * std::max(min_dist, 10.0f)
					)
				{
					continue;
				}

				// ����ƥ���
				matches.push_back(knn_matches[r][0]);
			}
		}
	};
}