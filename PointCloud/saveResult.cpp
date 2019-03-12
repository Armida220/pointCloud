#include "saveResult.h"

namespace calibration
{
	void saveCameraParams(
		const std::string& filename,
		const cv::Size& imageSize,
		const cv::Size& boardSize,
		const float& squareSize,
		const cv::Mat& cameraMatrix,
		const cv::Mat& distCoeffs,
		const std::vector<cv::Mat>& rvecs,
		const std::vector<cv::Mat>& tvecs,
		const std::vector<float>& reprojErrs,
		const std::vector<std::vector<cv::Point2f> >& imagePoints,
		const double& totalAvgErr)
	{
		std::cout << "start saving calibration result to file" << std::endl;
		cv::FileStorage fs(filename, cv::FileStorage::WRITE);

		// 记录标定时间
		struct tm t2;
		time_t tm;
		time(&tm);
		localtime_s(&t2, &tm);
		char buf[1024];
		strftime(buf, sizeof(buf), "%c", &t2);
		fs << "calibration_time" << buf;

		//time_t tt;
		//time(&tt);
		//struct tm *t2 = localtime(&tt);
		//fs << "calibration_time" << asctime(t2);


		if (!rvecs.empty() || !reprojErrs.empty())
			fs << "nr_of_frames" << (int)std::max(rvecs.size(), reprojErrs.size());
		// 记录标定参数
		fs << "image_width" << imageSize.width;
		fs << "image_height" << imageSize.height;
		fs << "board_width" << boardSize.width;
		fs << "board_height" << boardSize.height;
		fs << "square_size" << squareSize;

		// 记录标定结果
		fs << "camera_matrix" << cameraMatrix;
		fs << "distortion_coefficients" << distCoeffs;

		// 记录重投影误差
		fs << "avg_reprojection_error" << totalAvgErr;
		if (!reprojErrs.empty())
			fs << "per_view_reprojection_errors" << cv::Mat(reprojErrs);

		// 记录各个图片的外部矩阵
		if (!rvecs.empty() && !tvecs.empty())
		{
			// Checks revcs.type and tevcs.type at runtime and throws exception if it fails.
			CV_Assert(rvecs[0].type() == tvecs[0].type());

			cv::Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
			for (std::size_t i = 0; i < rvecs.size(); i++)
			{
				cv::Mat r = bigmat(cv::Range(int(i), int(i + 1)), cv::Range(0, 3));
				cv::Mat t = bigmat(cv::Range(int(i), int(i + 1)), cv::Range(3, 6));

				//*.t() is MatExpr (not Mat) so we can use assignment operator
				CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
				CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
				t = tvecs[i].t();
				r = rvecs[i].t();
			}
			fs.writeComment("a set of 6-tuples (rotation vector + translation vector) for each view", 0);
			fs << "extrinsic_parameters" << bigmat;
		}

		std::cout << "finished saving" << std::endl;
	}

	
}