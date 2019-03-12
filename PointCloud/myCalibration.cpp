#include "myCalibration.h"

using namespace cv;
using namespace std;


myCalibration::myCalibration()
{
}


myCalibration::~myCalibration()
{
}

/*
@param infile 目录文件的路径,其中保存了所有用于相机标定的文件名
@param K      储存相机内部矩阵
*/
void myCalibration::getIntrinsicMat(std::string infile, cv::Mat& K, cv::Mat& distCoeffs)
{
	cout << "开始检测角点................\n";

	ifstream fin(infile); 
	int imgCount = 0; // 标定用图像的总数
	Size imgSize; // 图像尺寸(默认所有图像尺寸一致，且由第一幅图像得到)
	Size boardSize = Size(BOARDSIZEX-1, BOARDSIZEY-1); // 棋盘每行每列的角点数
	vector<Point2f> imgPointsBuf; // 缓存一副图像上检测到的角点
	float squareSize = SQUARESIZE; // 每个棋盘格的真实大小
	vector<vector<Point3f>> objectPoints(1); // 保存角点的三维坐标
	vector<vector<Point2f>> imagePoints;  // 保存角点的二维坐标

	vector<Mat> rvecsMat;  // 存储所有图像的旋转向量
	vector<Mat> tvecsMat;  // 存储所有图像的平移向量


// ----------------------提取角点----------------------------	
	string filename; // 缓存图像名
	string filePosition; 
	while (getline(fin, filename))
	{
		cout << "imageCount = " << ++imgCount << endl;
		Mat imgInput = imread(CALIBRATION_IMG_DIR + filename);
		// 读取第一张图片的时候获取图像的宽高信息
		if (imgCount == 1) 
		{
			imgSize.height = imgInput.rows;
			imgSize.width = imgInput.cols;
			cout << "imgSize.height =" << imgSize.height << endl;
			cout << "imgSize.width ="  << imgSize.width << endl;
		}

		// 用findChessboardCorners()函数提取角点，保存到imgPointsBuf中
		if (!findChessboardCorners(imgInput, boardSize, imgPointsBuf, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
			+ CALIB_CB_FAST_CHECK))
		{
			cout << filename <<"中检测不到角点\n";
			exit(1);
		}
		else
		{
			Mat view_gray;
			cvtColor(imgInput, view_gray, COLOR_RGB2GRAY);
			
			cout << imgPointsBuf;
			// 亚像素精确化
			//find4QuadCornerSubpix(view_gray, imgPointsBuf, Size(11, 11)); 
//			TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
			cornerSubPix(view_gray, imgPointsBuf, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));


			imagePoints.push_back(imgPointsBuf);  //保存亚像素角点
			// 显示角点位置
			//drawChessboardCorners(view_gray, boardSize, imgPointsBuf, true);
			//imshow("Camera Calibration", view_gray);
			//waitKey(500);
		}
	}

	cout << "角点提取完成!\n";
	cout << "开始相机标定.........\n";
// -------------------------------------------------
	// 初始化标定板上角点的坐标
	calcBoardCornerPositions(boardSize, squareSize, objectPoints[0]);
	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	// 用calibrateCamera()计算相机的内部矩阵、每张图片相对于原点的平移矩阵和旋转矩阵
	calibrateCamera(objectPoints, imagePoints, imgSize, K, distCoeffs, rvecsMat, tvecsMat);
	

	cout << "\tK:\n\t" << K << endl;
	cout << "\tdistCoeffs:\n\t" << distCoeffs << endl;

	// 计算重投影误差
	vector<float> reprojErrs;
	float totalAvgErr = 0;
	computeReprojectionErrors(imgCount, objectPoints,
		rvecsMat, tvecsMat, K, distCoeffs, 
		imagePoints, reprojErrs, &totalAvgErr);



	cout << "标定结束\n";
	cout << "保存标定结果\n";
	// 保存到文件
	saveToFile("./result_1.yml", imgSize, boardSize, squareSize,
		K, distCoeffs,
		rvecsMat, tvecsMat,
		reprojErrs,
		imagePoints,
		&totalAvgErr);
	cout << "保存结束\n";

}

void  myCalibration::calcBoardCornerPositions(Size boardSize, float squareSize,
	vector<Point3f>& corners
)
{
	corners.clear();

	// 初始化标定板上角点的坐标，假设所有的角点都处于Z=0的平面上，且棋盘靠左上的第一个角点为（0，0，0）
	vector<Point3f> tempPointSet;
	for (int i = 0; i < boardSize.height; i++)
		for (int j = 0; j < boardSize.width; j++)
			corners.push_back(Point3f(j*squareSize, i*squareSize, 0));
}


void myCalibration::computeReprojectionErrors(
	int imgCount, 
	vector<vector<Point3f>> objectPoints,
	const vector<Mat>& rvecsMat,
	const vector<Mat>& tvecsMat,
	const Mat& K,
	const Mat& distCoeffs,
	const vector<vector<Point2f>>& imagePoints,
	vector<float>& reprojErrs,
	float* totalErrs
)
{
	cout << "\t开始评价标定结果………………\n";
	size_t totalPoints = 0;
	float err = 0.0;       
	vector<Point2f> image_points2; // 保存用内部矩阵重新计算得到的投影点
	cout << "\t\t每幅图像的标定误差：\n";
	for (int i = 0; i < imgCount; i++)
	{
		// 通过得到的摄像机内外参数，对棋盘的角点进行重新投影计算，保存到image_points2中
		projectPoints(objectPoints[i], rvecsMat[i], tvecsMat[i], K, distCoeffs, image_points2);

		// 计算重投影点和投影点之间的误差
		err = norm(imagePoints[i], image_points2, NORM_L2);

		size_t n = objectPoints[i].size();
		float perViewErr = (float)std::sqrt(err*err / n);
		cout << "\t\t\t第" << i + 1 << "幅图像的平均误差：" << perViewErr << "像素" << endl;
		reprojErrs.push_back(perViewErr);
		*totalErrs += err * err;
		totalPoints += n;
		//totalErr += err /= objectPoints[i].size();
		//cout << "\t\t\t第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
	}

	*totalErrs = sqrt(*totalErrs / totalPoints);
	cout << "\t\t总体平均误差：" << *totalErrs << "像素" << endl;
	
	
	cout << "\t评价完成\n";
}


void myCalibration::saveToFile(string outputFileName, Size imageSize, Size boardSize, float squareSize,
	Mat& cameraMatrix, Mat& distCoeffs,
	const vector<Mat>& rvecs, const vector<Mat>& tvecs,
	const vector<float>& reprojErrs, 
	const vector<vector<Point2f> >& imagePoints,
	float* totalAvgErr
)
{
	FileStorage fs(outputFileName, FileStorage::WRITE);

	struct tm t2;
	time_t tm;
	time(&tm);
	localtime_s(&t2, &tm);
	char buf[1024];
	strftime(buf, sizeof(buf), "%c", &t2);

	fs << "calibration_time" << buf;

	if (!rvecs.empty() || !reprojErrs.empty())
		fs << "nr_of_frames" << (int)std::max(rvecs.size(), reprojErrs.size());
	fs << "image_width" << imageSize.width;
	fs << "image_height" << imageSize.height;
	fs << "board_width" << boardSize.width;
	fs << "board_height" << boardSize.height;
	fs << "square_size" << squareSize;

	fs << "camera_matrix" << cameraMatrix;
	fs << "distortion_coefficients" << distCoeffs;

	fs << "avg_reprojection_error" << *totalAvgErr;
	//if (s.writeExtrinsics && !reprojErrs.empty())
	//	fs << "per_view_reprojection_errors" << Mat(reprojErrs);

	//if (s.writeExtrinsics && !rvecs.empty() && !tvecs.empty())
	//{
	//	CV_Assert(rvecs[0].type() == tvecs[0].type());
	//	Mat bigmat((int)rvecs.size(), 6, CV_MAKETYPE(rvecs[0].type(), 1));
	//	bool needReshapeR = rvecs[0].depth() != 1 ? true : false;
	//	bool needReshapeT = tvecs[0].depth() != 1 ? true : false;

	//	for (size_t i = 0; i < rvecs.size(); i++)
	//	{
	//		Mat r = bigmat(Range(int(i), int(i + 1)), Range(0, 3));
	//		Mat t = bigmat(Range(int(i), int(i + 1)), Range(3, 6));

	//		if (needReshapeR)
	//			rvecs[i].reshape(1, 1).copyTo(r);
	//		else
	//		{
	//			//*.t() is MatExpr (not Mat) so we can use assignment operator
	//			CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
	//			r = rvecs[i].t();
	//		}

	//		if (needReshapeT)
	//			tvecs[i].reshape(1, 1).copyTo(t);
	//		else
	//		{
	//			CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
	//			t = tvecs[i].t();
	//		}
	//	}
	//	fs.writeComment("a set of 6-tuples (rotation vector + translation vector) for each view");
	//	fs << "extrinsic_parameters" << bigmat;
	//}
}

