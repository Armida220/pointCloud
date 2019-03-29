#pragma once

#include <io.h>
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace imageIO
{

	/**
	 * @brief this function get all filenames under the given path
	 * @param[in] path 
	 * @param[out] fileList
	 * @param[in] fileType
	 */
	void getFiles(const std::string& path,  std::vector<std::string>& fileList, const std::string& fileType = "*");

	/**
	 * @brief this function read image like '*.jpg' from a given path and save them as cv::Mat into a vector 
	 * @param[in] path the given path
	 * @param[out] imageVec a vector consist of cv::Mat
	 */
	void readImage(const std::string& path, std::vector<cv::Mat>& imageVec );
}