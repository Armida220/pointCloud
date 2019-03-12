#pragma once


#include <vector>
#include <map>
#include <opencv2/opencv.hpp>

namespace calibration
{
	/**
	 * @brief This function compute the cell index of each image
	 *
	 *
	 */
	void computeCellIndexs(
		const std::vector<std::vector<cv::Point2f>>& imagePoints,
		const cv::Size& imageSize,
		const std::size_t& calibGridSize,
		std::vector<std::vector<std::size_t>>& cellIndexsPerImage);

	/**
	 * @brief This function compute each cell's weight from all images
     * 
	 *
	 */
	void computeCellsWeight(
		const std::vector<std::vector<std::size_t>>& cellIndexsPerImage,
		const std::size_t& calibGridSize,
		std::vector<std::size_t>& cellsWeight);

	/**
	 * @brief This function compute each image's score
	 */
	void computeImageScores(
		const std::vector<std::vector<std::size_t>>& cellIndexsPerImage,
		const std::vector<std::size_t>& cellsWeight,
		std::vector<float>& imageScores);

}