#include "projection.h"
#include <opencv2/calib3d/calib3d.hpp>

namespace multiView
{


	void project(

	)
	{
		return;
	}

	void triangulateViews(
		const cv::Mat& K,
		const cv::Mat& R1,
		const cv::Mat& T1,
		const cv::Mat& R2,
		const cv::Mat& T2,
		const std::vector<cv::Point2f>& p1,
		const std::vector<cv::Point2f>& p2,
		//std::vector<cv::Point3f>& structure
		std::vector<cv::Point3d>& structure
	)
	{
		//两个相机的投影矩阵[R T]，triangulatePoints只支持float型
		cv::Mat proj1(3, 4, CV_32FC1);
		cv::Mat proj2(3, 4, CV_32FC1);

		R1.convertTo(proj1(cv::Range(0, 3), cv::Range(0, 3)), CV_32FC1);
		T1.convertTo(proj1.col(3), CV_32FC1);

		R2.convertTo(proj2(cv::Range(0, 3), cv::Range(0, 3)), CV_32FC1);
		T2.convertTo(proj2.col(3), CV_32FC1);

		cv::Mat fK;
		K.convertTo(fK, CV_32FC1);
		proj1 = fK * proj1;
		proj2 = fK * proj2;

		//三角重建
		cv::Mat s;
		cv::triangulatePoints(proj1, proj2, p1, p2, s);

		structure.clear();
		structure.reserve(s.cols);
		for (int i = 0; i < s.cols; ++i)
		{
			cv::Mat_<float> col = s.col(i);
			col /= col(3);  //齐次坐标，需要除以最后一个元素才是真正的坐标值
			structure.push_back(cv::Point3f(col(0), col(1), col(2)));
		}
	}

	double reprojectionErrorRMSE(

	)
	{
		return 0;
	}

}