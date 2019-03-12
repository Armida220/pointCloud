#include "evaluateImages.h"

namespace calibration
{
	void computeCellIndexs(
		const std::vector<std::vector<cv::Point2f>>& imagePoints,
		const cv::Size& imageSize,
		const std::size_t& calibGridSize,
		std::vector<std::vector<std::size_t>>& cellIdxPerImage)
	{
		// ����ÿ����Ԫ��Ŀ��
		float cellWidth = float(imageSize.width) / float(calibGridSize);
		float cellHeight = float(imageSize.height) / float(calibGridSize);

		// ����ÿ��ͼ����ͼ���ĵ�Ԫ���
		for (const std::vector<cv::Point2f>& pointBuf : imagePoints)
		{
			std::vector<std::size_t> imageCellIdx;
			for (cv::Point2f point : pointBuf)
			{

				std::size_t cellX = std::floor(point.x / cellWidth);
				std::size_t cellY = std::floor(point.y / cellHeight);
				std::size_t cellIdx = cellY * calibGridSize + cellX;
				imageCellIdx.push_back(cellIdx);
			}
			cellIdxPerImage.push_back(imageCellIdx);
		}
	}

	void computeCellsWeight(
		const std::vector<std::vector<std::size_t>>& cellIndexsPerImage,
		const std::size_t& calibGridSize,
		std::vector<std::size_t>& cellsWeight)
	{
		// ��0��ʼ��
		cellsWeight.resize(calibGridSize*calibGridSize);
		for (int i = 0; i < calibGridSize*calibGridSize; ++i)
		{
			cellsWeight[i] = 0;
		}

		// ����ÿ��ͼ�񣬼���Ȩ��
		for (const std::vector<std::size_t>& imageCellIdx : cellIndexsPerImage)
		{
			// ��ȥ��ͬ��idx
			std::vector<std::size_t> uniqueImageCellIdx = imageCellIdx;
			std::sort(uniqueImageCellIdx.begin(), uniqueImageCellIdx.end());
			auto last = std::unique(uniqueImageCellIdx.begin(), uniqueImageCellIdx.end());
			uniqueImageCellIdx.erase(last, uniqueImageCellIdx.end());

			for (std::size_t cellIdx : uniqueImageCellIdx)
			{
				cellsWeight[cellIdx]++;
			}
		}
	}

	void computeImageScores(
		const std::vector<std::vector<std::size_t>>& cellIndexsPerImage,
		const std::vector<std::size_t>& cellsWeight,
		std::vector<float>& imageScores)
	{
		// ����ÿ��ͼƬ�ĵ÷�
		for (const std::vector<std::size_t>& imageCellIdx : cellIndexsPerImage)
		{
			float imageScore = 0;
			for (std::size_t cellIdx : imageCellIdx)
			{
				imageScore += cellsWeight[cellIdx];
			}

			imageScore /= float(imageCellIdx.size());
			imageScores.push_back(imageScore);
		}

	}





}