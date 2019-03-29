#include "epipolar.h"
#include <opencv2/opencv.hpp>

namespace multiView
{
	int estimateEMatrix(
		const cv::Mat& K,
		const std::vector<cv::Point2f>& p1,
		const std::vector<cv::Point2f>& p2,
		cv::Mat& E,
		cv::Mat& mask
	)
	{

		//����ƥ�����ȡ��������ʹ��RANSAC����һ���ų�ʧ���
		E = cv::findEssentialMat(p1, p2, K, cv::RANSAC, 0.999, 1.0, mask);
		if (E.empty()) return -1;

		double feasible_count = countNonZero(mask);
		std::cout << (int)feasible_count << " -in- " << p1.size() << std::endl;
		//����RANSAC���ԣ�outlier��������50%ʱ������ǲ��ɿ���
		if (feasible_count <= 15 || (feasible_count / p1.size()) < 0.6)
			return -1;

		return feasible_count;
	}


	int estimateFMatrix(
		const std::vector<cv::Point2f>& p1,
		const std::vector<cv::Point2f>& p2,
		cv::Mat& F,
		cv::Mat& mask
	)
	{

		F = cv::findFundamentalMat(p1, p2, cv::RANSAC, 3.0, 0.99, mask);

		if (F.empty()) return -1;

		double feasible_count = countNonZero(mask);
		std::cout << (int)feasible_count << " -in- " << p1.size() << std::endl;
		//����RANSAC���ԣ�outlier��������50%ʱ������ǲ��ɿ���
		if (feasible_count <= 15 || (feasible_count / p1.size()) < 0.6)
			return -1;

		return feasible_count;
	}

	bool estimatePose(
		const cv::Mat& K,
		const std::vector<cv::Point2f>& p1,
		const std::vector<cv::Point2f>& p2,
		cv::Mat& R,
		cv::Mat& T,
		cv::Mat& mask
	)
	{



		//����ƥ�����ȡ��������ʹ��RANSAC����һ���ų�ʧ���
		cv::Mat E;
		double feasible_count;
		feasible_count = estimateEMatrix(K, p1, p2, E, mask);
		std::cout << E << std::endl;


		std::cout << p1.size() << ", " << p2.size() << std::endl;

		//�ֽⱾ�����󣬻�ȡ��Ա任
		int pass_count = cv::recoverPose(E, p1, p2, K, R, T, mask);

		//ͬʱλ���������ǰ���ĵ������Ҫ�㹻��
		if (((double)pass_count) / feasible_count < 0.7)
			return false;

		return true;
	}

}