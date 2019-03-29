#include "stereo.h"
#include "epipolar.h"
#include "projection.h"
#include "common.h"

#include <iostream>

//#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

namespace multiView
{
	void findBaselineTriangulation(
		const cv::Mat K,
		const std::vector<std::vector<cv::KeyPoint>>& key_points_for_all,
		const std::vector<std::vector<cv::Vec3b>>& colors_for_all,
		const std::vector<std::vector<cv::DMatch>>& matches_for_all,
		std::vector<cv::Point3d>& structure,
		//std::vector<cv::Point3f>& structure,
		std::vector<std::vector<int>>& correspond_struct_idx,
		std::vector<cv::Vec3b>& colors,
		std::vector<cv::Mat>& rotations,
		std::vector<cv::Mat>& motions
	)
	{
		//����ͷ����ͼ��֮��ı任����
		std::vector<cv::Point2f> p1;
		std::vector<cv::Point2f> p2;
		std::vector<cv::Vec3b> c2;
		cv::Mat R, T, mask;

		getMatchedPoints(key_points_for_all[0], key_points_for_all[1], matches_for_all[0], p1, p2);
		getMatchedColors(colors_for_all[0], colors_for_all[1], matches_for_all[0], colors, c2);
		
		estimatePose(K, p1, p2, R, T, mask);

		//��ͷ����ͼ�������ά�ؽ�
		maskoutPoints(p1, mask);
		maskoutPoints(p2, mask);
		maskoutColors(colors, mask);

		cv::Mat R0 = cv::Mat::eye(3, 3, CV_64FC1);
		cv::Mat T0 = cv::Mat::zeros(3, 1, CV_64FC1);
		triangulateViews(K, R0, T0, R, T, p1, p2, structure);
		//����任����
		rotations = { R0, R };
		motions = { T0, T };

		//��correspond_struct_idx�Ĵ�С��ʼ��Ϊ��key_points_for_all��ȫһ��
		correspond_struct_idx.clear();
		correspond_struct_idx.resize(key_points_for_all.size());
		for (int i = 0; i < key_points_for_all.size(); ++i)
		{
			correspond_struct_idx[i].resize(key_points_for_all[i].size(), -1);
		}

		//��дͷ����ͼ��Ľṹ����
		int idx = 0;
		std::vector<cv::DMatch> matches = matches_for_all[0];
		for (int i = 0; i < matches.size(); ++i)
		{
			if (mask.at<uchar>(i) == 0)
				continue;

			correspond_struct_idx[0][matches[i].queryIdx] = idx;
			correspond_struct_idx[1][matches[i].trainIdx] = idx;
			++idx;
		}
	}

	void get_objpoints_and_imgpoints(
		const std::vector<cv::DMatch>& matches,
		const std::vector<int>& struct_indices,
		//const std::vector<cv::Point3f>& structure,
		const std::vector<cv::Point3d>& structure,
		const std::vector<cv::KeyPoint>& key_points,
		std::vector<cv::Point3f>& object_points,
		std::vector<cv::Point2f>& image_points
	)
	{
		object_points.clear();
		image_points.clear();

		for (int i = 0; i < matches.size(); ++i)
		{
			int queryIdx = matches[i].queryIdx;
			int trainIdx = matches[i].trainIdx;

			int structIdx = struct_indices[queryIdx];
			if (structIdx < 0) continue;

			object_points.push_back(structure[structIdx]);
			image_points.push_back(key_points[trainIdx].pt);
		}
	}

	void fusion_structure(
		const std::vector<cv::DMatch>& matches,
		std::vector<int>& struct_indices,
		std::vector<int>& next_struct_indices,
		//std::vector<cv::Point3f>& structure,
		std::vector<cv::Point3d>& structure,
		//std::vector<cv::Point3f>& next_structure,
		std::vector<cv::Point3d>& next_structure,
		std::vector<cv::Vec3b>& colors,
		std::vector<cv::Vec3b>& next_colors
	)
	{
		for (int i = 0; i < matches.size(); ++i)
		{
			int queryIdx = matches[i].queryIdx;
			int trainIdx = matches[i].trainIdx;

			int structIdx = struct_indices[queryIdx];

			// ���õ��ڿռ����Ѿ����ڣ������ƥ����Ӧ�Ŀռ��Ӧ����ͬһ��������Ҫ��ͬ
			if (structIdx >= 0)
			{
				next_struct_indices[trainIdx] = structIdx;
				continue;
			}

			// ���õ��ڿռ��в����ڣ����õ���뵽�ṹ�У������ƥ���Ŀռ��������Ϊ�¼���ĵ������
			structure.push_back(next_structure[i]);
			colors.push_back(next_colors[i]);
			struct_indices[queryIdx] = next_struct_indices[trainIdx] = structure.size() - 1;
		}
	}

	void generalRebuild(
		const cv::Mat& K,
		const cv::Mat& distCoeffs,
		const std::vector<std::vector<cv::KeyPoint>>& key_points_for_all,
		const std::vector<std::vector<cv::Vec3b>>& colors_for_all,
		const std::vector<std::vector<cv::DMatch>>& matches_for_all,
		//std::vector<cv::Point3f>& structure,
		std::vector<cv::Point3d>& structure,
		std::vector<std::vector<int>>& correspond_struct_idx,
		std::vector<cv::Vec3b>& colors,
		std::vector<cv::Mat>& rotations,
		std::vector<cv::Mat>& motions
	)
	{
		//��ʼ���ṹ����ά���ƣ�
		findBaselineTriangulation(
			K,
			key_points_for_all,
			colors_for_all,
			matches_for_all,
			structure,
			correspond_struct_idx,
			colors,
			rotations,
			motions
		);

		//������ʽ�ؽ�ʣ���ͼ��
		for (int i = 1; i < matches_for_all.size(); ++i)
		{
			std::vector<cv::Point3f> object_points;
			std::vector<cv::Point2f> image_points;
			cv::Mat r, R, T;
			//Mat mask;

			//��ȡ��i��ͼ����ƥ����Ӧ����ά�㣬�Լ��ڵ�i+1��ͼ���ж�Ӧ�����ص�
			get_objpoints_and_imgpoints(
				matches_for_all[i],
				correspond_struct_idx[i],
				structure,
				key_points_for_all[i + 1],
				object_points,
				image_points
			);

			//���任����
			//cv::solvePnPRansac(object_points, image_points, K, distCoeffs, r, T);
			cv::solvePnPRansac(object_points, image_points, K, cv::noArray(), r, T);
			//����ת����ת��Ϊ��ת����
			std::cout << r <<  std::endl;

			cv::Rodrigues(r, R);
			//����任����
			rotations.push_back(R);
			motions.push_back(T);

			std::vector<cv::Point2f> p1, p2;
			std::vector<cv::Vec3b> c1, c2;
			getMatchedPoints(key_points_for_all[i], key_points_for_all[i + 1], matches_for_all[i], p1, p2);
			getMatchedColors(colors_for_all[i], colors_for_all[i + 1], matches_for_all[i], c1, c2);

			//����֮ǰ��õ�R��T������ά�ؽ�
			//std::vector<cv::Point3f> next_structure;
			std::vector<cv::Point3d> next_structure;
			triangulateViews(K, rotations[i], motions[i], R, T, p1, p2, next_structure);

			//���µ��ؽ������֮ǰ���ں�
			fusion_structure(
				matches_for_all[i],
				correspond_struct_idx[i],
				correspond_struct_idx[i + 1],
				structure,
				next_structure,
				colors,
				c1
			);
		}
	}

}