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
//	// --------------------------- ����궨ģ�� -----------------
//		// ��������ڲ�����;�ͷ�������
//	Mat K = Mat(3, 3, CV_32FC1, Scalar::all(0));
//	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));
//	myCalibration::getIntrinsicMat(CALIBRATION_CATALOG_FILE, K, distCoeffs);
//
//
//	// ����������ڲ�����
//	//Mat K(Matx33d(
//	//	3071.477, 0, 1066.067,
//	//	0, 3069.834, 1929.784,
//	//	0, 0, 1));
//
//// ---------------------------- SFMģ�� -----------------------
//	// ��ȡ�ؽ��õ�ͼ���ļ���
//	//getFiles(CONTRIBUTE_IMG_DIR, img_names);
//	//
//	//vector<string>::iterator iter;
//	//for (iter = img_names.begin(); iter != img_names.end(); iter++)
//	//{
//	//	cout << *iter << endl;
//	//}
//
//	//// ��ȡ����ͼ�������������������
//	//extract_features(img_names, key_points_for_all, descriptor_for_all, colors_for_all);
//	//// ������ͼ�����˳�ε�����ƥ��
//	//match_features(descriptor_for_all, matches_for_all);
//
//	//// ��ʼ���ṹ����ά���ƣ�ͷ����
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
