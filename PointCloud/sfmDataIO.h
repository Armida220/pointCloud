#pragma once
#include <fstream>

#include <opencv2/core/core.hpp>

namespace sfm
{
	/**
	 * @brief this function save structure to a yml file
	 * @param[in] fileName
	 * @param[in] rotations
	 * @param[in] motions
	 * @param[in] structure
	 * @param[in] colors
	 */
	void save2Yml(
		const std::string& file_name,
		const std::vector<cv::Mat>& rotations,
		const std::vector<cv::Mat>& motions,
		//const std::vector<cv::Point3f>& structure,
		const std::vector<cv::Point3d>& structure,
		const std::vector<cv::Vec3b>& colors
	);

	/**
	 * @brief this function save structure to a ply file
	 * @param[in] fileName
	 * @param[in] rotations
	 * @param[in] motions
	 * @param[in] structure
	 * @param[in] colors
	 */
	bool save2Ply(
		const std::string& filename,
		const std::vector<cv::Point3d>& structure,
		const std::vector<cv::Vec3b>& colors
	);


}