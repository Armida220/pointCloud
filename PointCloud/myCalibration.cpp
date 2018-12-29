#include "myCalibration.h"



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
void myCalibration::getIntrinsicMat(std::string infile, cv::Mat& K)
{

	using namespace cv;
	using namespace std;

    cout << "running getIntrinsicMat function!\n";
	cout << "开始检测角点................\n";

	ifstream fin(infile); 
	int imgCount = 0; // 标定用图像的总数
	Size imgSize; // 图像尺寸(默认所有图像尺寸一致，且由第一幅图像得到)
	Size boardSize = Size(BOARDSIZEX, BOARDSIZEY); // 棋盘每行每列的角点数
	vector<Point2f> imgPointsBuf; // 缓存一副图像上检测到的角点
	vector<vector<Point2f>> imgPointsSeq; // 储存所有图像上的角点
	

// 提取角点#########################################################	
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
		if (!findChessboardCorners(imgInput, boardSize, imgPointsBuf))
		{
			cout << filename <<"中检测不到角点";
			exit(1);
		}
		else
		{
			Mat view_gray;
			cvtColor(imgInput, view_gray, COLOR_RGB2GRAY);
			// 亚像素精确化
			find4QuadCornerSubpix(view_gray, imgPointsBuf, Size(11, 11)); //对粗提取的角点进行精确化
			imgPointsSeq.push_back(imgPointsBuf);  //保存亚像素角点
			// 显示角点位置
			//drawChessboardCorners(view_gray, boardSize, imgPointsBuf, true);
			//imshow("Camera Calibration", view_gray);
			//waitKey(500);
		}
	}
	cout << "角点提取完成" << endl;

// 相机标定##############################################################
	cout << "开始相机标定.........\n";

	Size squareSize = Size(SQUARESIZEX, SQUARESIZEY); // 每个棋盘格的真实大小
	vector<vector<Point3f>> objectPoints; // 保存角点的三维坐标
	vector<int> pointCounts;  // 每幅图像中角点的数量
	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0)); // 摄像机的5个畸变系数：k1,k2,p1,p2,k3
	vector<Mat> rvecsMat;  // 存储每幅图像的旋转向量
	vector<Mat> tvecsMat;  // 存储每幅图像的平移向量 

	// 初始化标定板上角点的坐标，假设所有的角点都处于Z=0的平面上，且棋盘靠左上的第一个角点为（0，0，0）
	int i, j, t;
	for (t = 0; t < imgCount; t++)
	{
		vector<Point3f> tempPointSet;
		for (i = 0; i < boardSize.height; i++)
		{
			for (j = 0; j < boardSize.width; j++)
			{
				Point3f realPoint;
				// 假设标定板放在世界坐标系中z=0的平面上 
				realPoint.x = i * squareSize.width;
				realPoint.y = j * squareSize.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		objectPoints.push_back(tempPointSet);
		// 初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板
		pointCounts.push_back(boardSize.width*boardSize.height);
	}
	// 用calibrateCamera()计算相机的内部矩阵，每张图片相对于原点的平移矩阵和旋转矩阵
	calibrateCamera(objectPoints, imgPointsSeq, imgSize, K, distCoeffs, rvecsMat, tvecsMat);
	cout << "内部矩阵K:\n " << K << endl;


//对标定结果进行评价#######################################
	cout << "开始评价标定结果………………\n";
	double total_err = 0.0; // 所有图像的平均误差的总和
	double err = 0.0; // 每幅图像的平均误差
	vector<Point2f> image_points2; // 保存用内部矩阵重新计算得到的投影点
	cout << "\t每幅图像的标定误差：\n";
	for (i = 0; i < imgCount; i++)
	{
		vector<Point3f> tempPointSet = objectPoints[i];
		// 通过得到的摄像机内外参数，对棋盘的角点进行重新投影计算，保存到image_points2中
		projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], K, distCoeffs, image_points2);
		
		// 计算新的投影点和旧的投影点之间的误差
		vector<Point2f> tempImagePoint = imgPointsSeq[i];
		Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
		for (int j = 0; j < tempImagePoint.size(); j++)
		{
			image_points2Mat.at<Vec2f>(0, j) = Vec2f(image_points2[j].x, image_points2[j].y);
			tempImagePointMat.at<Vec2f>(0, j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
		}
		err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
		total_err += err /= pointCounts[i];
		cout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
	}
	cout << "总体平均误差：" << total_err / imgCount << "像素" << endl;
	cout << "评价完成\n";
	cout << "finish getIntrinsicMat function!\n\n\n";
}

