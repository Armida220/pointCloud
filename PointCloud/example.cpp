#include <io.h>
#include "myCalibration.h"
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

using namespace cv;
using namespace std;


//int main()
//{
//
//	// --------------------------- 相机标定模块 -----------------
//		// 计算相机内部矩阵和镜头畸变参数
//	Mat K = Mat(3, 3, CV_32FC1, Scalar::all(0));
//	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));
//	myCalibration::getIntrinsicMat(CALIBRATION_CATALOG_FILE, K, distCoeffs);
//
//
//	// 给定的相机内部矩阵
//	//Mat K(Matx33d(
//	//	3071.477, 0, 1066.067,
//	//	0, 3069.834, 1929.784,
//	//	0, 0, 1));
//
//// ---------------------------- SFM模块 -----------------------
//	// 获取重建用的图像文件名
//	//getFiles(CONTRIBUTE_IMG_DIR, img_names);
//	//
//	//vector<string>::iterator iter;
//	//for (iter = img_names.begin(); iter != img_names.end(); iter++)
//	//{
//	//	cout << *iter << endl;
//	//}
//
//	//// 提取所有图像的特征并计算描述子
//	//extract_features(img_names, key_points_for_all, descriptor_for_all, colors_for_all);
//	//// 对所有图像进行顺次的特征匹配
//	//match_features(descriptor_for_all, matches_for_all);
//
//	//// 初始化结构（三维点云）头两幅
//	//init_structure(
//	//	K,
//	//	key_points_for_all,
//	//	colors_for_all,
//	//	matches_for_all,
//	//	structure,
//	//	correspond_struct_idx,
//	//	colors,
//	//	rotations,
//	//	motions
//	//);
//
//	cout << "successful!!!" << endl;
//
//	waitKey(0);
//}
