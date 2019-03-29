#include "sfm.h"

int main()
{

	// --------------------------- Calibration -----------------
		// 计算相机内部矩阵和镜头畸变参数
	//Mat K = Mat(3, 3, CV_32FC1, Scalar::all(0));
	//Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));
	//myCalibration::getIntrinsicMat(CALIBRATION_CATALOG_FILE, K, distCoeffs);


	// --------------------------- SfM -----------------
	sfm::SfM sfmSlover;
	sfmSlover.initSfM(".\\sfm_config.xml");
	sfmSlover.runSfM();

}
