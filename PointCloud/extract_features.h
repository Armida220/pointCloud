#pragma once
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "def.h"

using namespace std;
using namespace cv;

namespace gg
{

	void extract_features(
		vector<string>& image_names,
		vector<vector<KeyPoint>>& key_points_for_all,
		vector<Mat>& descriptor_for_all,
		vector <vector<Vec3b>>& colors_for_all
	);
}