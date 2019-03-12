#include "patternDetect.h"
#include <opencv2/calib3d.hpp>


namespace calibration
{

	/**
	 * @brief This function get Patterns' coordinates from input image
	 *
	 */
	bool findPattern(
		const cv::Mat& viewGray,
		const cv::Size& boardSize,
		std::vector<cv::Point2f>& pointBuf)
	{
		pointBuf.clear();

		bool found = false;
		found = cv::findChessboardCorners(viewGray, boardSize, pointBuf, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

		if (found)
		{
			cv::cornerSubPix(viewGray, pointBuf, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
		}

		return found;

	}


	void calcChessboardCorners(
		const cv::Size& boardSize,
		const float squareSize,
		std::vector<cv::Point3f>& corners)
	{
		corners.resize(0);

		// 初始化标定板上角点的坐标，假设所有的角点都处于Z=0的平面上，且棋盘靠左上的第一个角点为（0，0，0）
		for (int y = 0; y < boardSize.height; ++y)
			for (int x = 0; x < boardSize.width; ++x)
				corners.push_back(cv::Point3f(float(x * squareSize), float(y * squareSize), 0));
	}


	void computeObjectPoints(
		const cv::Size& boardSize,
		const float squareSize,
		const std::vector<std::vector<cv::Point2f>>& imagePoints,
		std::vector<std::vector<cv::Point3f>>& objectPoints)
	{

		std::vector<cv::Point3f> tempObjectPoints;

		// Generate the object points coordinates
		calcChessboardCorners(boardSize, squareSize, tempObjectPoints);

		// Assign the corners to all items
		objectPoints.resize(imagePoints.size(), tempObjectPoints);
	}

}