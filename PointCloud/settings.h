#pragma once
#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

namespace calibration
{
	class Settings
	{

	public:
		Settings() : goodInput(false) {}
		enum InputType { INVALID, IMAGE_LIST };

		//void write(cv::FileStorage& fs)                       //Write serialization for this class
		//{
		//	fs << "{"
		//		<< "BoardSize_Width" << boardSize.width
		//		<< "BoardSize_Height" << boardSize.height
		//		<< "Square_Size" << squareSize
		//		<< "Calibrate_Pattern" << patternToUse
		//		<< "Calibrate_NrOfFrameToUse" << nrFrames
		//		<< "Calibrate_FixAspectRatio" << aspectRatio
		//		<< "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
		//		<< "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

		//		<< "Write_DetectedFeaturePoints" << writePoints
		//		<< "Write_extrinsicParameters" << writeExtrinsics
		//		<< "Write_gridPoints" << writeGrid
		//		<< "Write_outputFileName" << outputFileName

		//		<< "Show_UndistortedImage" << showUndistorsed

		//		<< "Input_FlipAroundHorizontalAxis" << flipVertical
		//		<< "Input_Delay" << delay
		//		<< "Input" << input
		//		<< "}";
		//}

		bool read(const cv::FileNode& node)                          //Read serialization for this class
		{
			std::cout << "reading...\n";
			node["BoardSize_Width"] >> boardSize.width;
			node["BoardSize_Height"] >> boardSize.height;
			node["ImageSize_Width"] >> imageSize.width;
			node["ImageSize_Height"] >> imageSize.height;
			//node["Calibrate_Pattern"] >> patternToUse;
			node["Square_Size"] >> squareSize;
			//node["Calibrate_NrOfFrameToUse"] >> nrFrames;
			//node["Calibrate_FixAspectRatio"] >> aspectRatio;
			//node["Write_DetectedFeaturePoints"] >> writePoints;
			//node["Write_extrinsicParameters"] >> writeExtrinsics;
			//node["Write_gridPoints"] >> writeGrid;
			node["Write_outputFileName"] >> outputFileName;
			//node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
			//node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
			//node["Calibrate_UseFisheyeModel"] >> useFisheye;
			//node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
			//node["Show_UndistortedImage"] >> showUndistorsed;
			node["Calibration_Image_Dir"] >> calibImgDir;
			node["Input"] >> input;
			///node["Input_Delay"] >> delay;
			//node["Fix_K1"] >> fixK1;
			//node["Fix_K2"] >> fixK2;
			//node["Fix_K3"] >> fixK3;
			//node["Fix_K4"] >> fixK4;
			//node["Fix_K5"] >> fixK5;

			return validate();
		}

		bool validate()
		{
			goodInput = true;
			boardSize.width--;
			boardSize.height--;
			if (boardSize.width <= 0 || boardSize.height <= 0)
			{
				std::cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << std::endl;
				goodInput = false;
			}
			if (squareSize <= 10e-6)
			{
				std::cerr << "Invalid square size " << squareSize << std::endl;
				goodInput = false;
			}
			//if (nrFrames <= 0)
			//{
			//	cerr << "Invalid number of frames " << nrFrames << endl;
			//	goodInput = false;
			//}

			if (input.empty())      // Check for valid input
				//goodInput = false;
				inputType = calibration::Settings::InputType::INVALID;
			else
			{
				//if (input[0] >= '0' && input[0] <= '9')
				//{
				//	stringstream ss(input);
				//	ss >> cameraID;
				//	inputType = CAMERA;
				//}
				//else
				//{
				if (isListOfImages(input) && readStringList(input, imageList))
				{
					inputType = calibration::Settings::InputType::IMAGE_LIST;
					//nrFrames = (nrFrames < (int)imageList.size()) ? nrFrames : (int)imageList.size();
				}
				//	else
				//		inputType = VIDEO_FILE;
				//}
				//if (inputType == CAMERA)
				//	inputCapture.open(cameraID);
				//if (inputType == VIDEO_FILE)
				//	inputCapture.open(input);
				//if (inputType != IMAGE_LIST && !inputCapture.isOpened())
				//	inputType = INVALID;
			}
			if (inputType == calibration::Settings::InputType::INVALID)
			{
				std::cerr << " Input does not exist: " << input;
				goodInput = false;
			}

			//flag = 0;
			//if (calibFixPrincipalPoint) flag |= CALIB_FIX_PRINCIPAL_POINT;
			//if (calibZeroTangentDist)   flag |= CALIB_ZERO_TANGENT_DIST;
			//if (aspectRatio)            flag |= CALIB_FIX_ASPECT_RATIO;
			//if (fixK1)                  flag |= CALIB_FIX_K1;
			//if (fixK2)                  flag |= CALIB_FIX_K2;
			//if (fixK3)                  flag |= CALIB_FIX_K3;
			//if (fixK4)                  flag |= CALIB_FIX_K4;
			//if (fixK5)                  flag |= CALIB_FIX_K5;

			//if (useFisheye) {
			//	// the fisheye model has its own enum, so overwrite the flags
			//	flag = fisheye::CALIB_FIX_SKEW | fisheye::CALIB_RECOMPUTE_EXTRINSIC;
			//	if (fixK1)                   flag |= fisheye::CALIB_FIX_K1;
			//	if (fixK2)                   flag |= fisheye::CALIB_FIX_K2;
			//	if (fixK3)                   flag |= fisheye::CALIB_FIX_K3;
			//	if (fixK4)                   flag |= fisheye::CALIB_FIX_K4;
			//	if (calibFixPrincipalPoint) flag |= fisheye::CALIB_FIX_PRINCIPAL_POINT;
			//}

			//calibrationPattern = NOT_EXISTING;
			//if (!patternToUse.compare("CHESSBOARD")) calibrationPattern = CHESSBOARD;
			//if (!patternToUse.compare("CIRCLES_GRID")) calibrationPattern = CIRCLES_GRID;
			//if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID")) calibrationPattern = ASYMMETRIC_CIRCLES_GRID;
			//if (calibrationPattern == NOT_EXISTING)
			//{
			//	cerr << " Camera calibration mode does not exist: " << patternToUse << endl;
			//	goodInput = false;
			//}
			//atImageList = 0;

			return goodInput;

		}


		static bool readStringList(const std::string& filename, std::vector<std::string>& list)
		{
			//return false;
			list.clear();
			cv::FileStorage fs(filename, cv::FileStorage::READ);
			if (!fs.isOpened())
				return false;
			cv::FileNode n = fs.getFirstTopLevelNode();
			if (n.type() != cv::FileNode::SEQ)
				return false;
			cv::FileNodeIterator it = n.begin(), it_end = n.end();
			for (; it != it_end; ++it)
				list.push_back((std::string)*it);
			return true;
		}


		static bool isListOfImages(const std::string& filename)
		{
			std::string s(filename);
			// Look for file extension
			if (s.find(".xml") == std::string::npos && s.find(".yaml") == std::string::npos && s.find(".yml") == std::string::npos)
				return false;
			else
				return true;
		}


	public:
		cv::Size boardSize;              // The size of the board -> Number of items by width and height
		cv::Size imageSize;
		//Pattern calibrationPattern;  // One of the Chessboard, circles, or asymmetric circle pattern
		float squareSize;            // The size of a square in your defined unit (point, millimeter,etc).
		//int nrFrames;                // The number of frames to use from the input for calibration
		//float aspectRatio;           // The aspect ratio
		//int delay;                   // In case of a video input
		//bool writePoints;            // Write detected feature points
		//bool writeExtrinsics;        // Write extrinsic parameters
		//bool writeGrid;              // Write refined 3D target grid points
		//bool calibZeroTangentDist;   // Assume zero tangential distortion
		//bool calibFixPrincipalPoint; // Fix the principal point at the center
		//bool flipVertical;           // Flip the captured images around the horizontal axis
		std::string outputFileName;       // The name of the file where to write
		//bool showUndistorsed;        // Show undistorted images after calibration
		std::string input;                // The input ->
		std::string calibImgDir;
		//bool useFisheye;             // use fisheye camera model for calibration
		//bool fixK1;                  // fix K1 distortion coefficient
		//bool fixK2;                  // fix K2 distortion coefficient
		//bool fixK3;                  // fix K3 distortion coefficient
		//bool fixK4;                  // fix K4 distortion coefficient
		//bool fixK5;                  // fix K5 distortion coefficient

		//int cameraID;
		std::vector<std::string> imageList;
		//size_t atImageList;
		//VideoCapture inputCapture;
		calibration::Settings::InputType inputType;
		bool goodInput;
		int flag;

	};

	static inline void read(const cv::FileNode& node, Settings& x, const Settings& default_value = Settings())
	{
		if (node.empty())
			x = default_value;
		else
			x.read(node);
	}

}