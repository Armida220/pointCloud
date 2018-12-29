#include "myCalibration.h"



myCalibration::myCalibration()
{
}


myCalibration::~myCalibration()
{
}


/*
@param infile Ŀ¼�ļ���·��,���б�����������������궨���ļ���
@param K      ��������ڲ�����
*/
void myCalibration::getIntrinsicMat(std::string infile, cv::Mat& K)
{

	using namespace cv;
	using namespace std;

    cout << "running getIntrinsicMat function!\n";
	cout << "��ʼ���ǵ�................\n";

	ifstream fin(infile); 
	int imgCount = 0; // �궨��ͼ�������
	Size imgSize; // ͼ��ߴ�(Ĭ������ͼ��ߴ�һ�£����ɵ�һ��ͼ��õ�)
	Size boardSize = Size(BOARDSIZEX, BOARDSIZEY); // ����ÿ��ÿ�еĽǵ���
	vector<Point2f> imgPointsBuf; // ����һ��ͼ���ϼ�⵽�Ľǵ�
	vector<vector<Point2f>> imgPointsSeq; // ��������ͼ���ϵĽǵ�
	

// ��ȡ�ǵ�#########################################################	
	string filename; // ����ͼ����
	string filePosition; 
	while (getline(fin, filename))
	{
		cout << "imageCount = " << ++imgCount << endl;
		Mat imgInput = imread(CALIBRATION_IMG_DIR + filename);
		// ��ȡ��һ��ͼƬ��ʱ���ȡͼ��Ŀ����Ϣ
		if (imgCount == 1) 
		{
			imgSize.height = imgInput.rows;
			imgSize.width = imgInput.cols;
			cout << "imgSize.height =" << imgSize.height << endl;
			cout << "imgSize.width ="  << imgSize.width << endl;
		}

		// ��findChessboardCorners()������ȡ�ǵ㣬���浽imgPointsBuf��
		if (!findChessboardCorners(imgInput, boardSize, imgPointsBuf))
		{
			cout << filename <<"�м�ⲻ���ǵ�";
			exit(1);
		}
		else
		{
			Mat view_gray;
			cvtColor(imgInput, view_gray, COLOR_RGB2GRAY);
			// �����ؾ�ȷ��
			find4QuadCornerSubpix(view_gray, imgPointsBuf, Size(11, 11)); //�Դ���ȡ�Ľǵ���о�ȷ��
			imgPointsSeq.push_back(imgPointsBuf);  //���������ؽǵ�
			// ��ʾ�ǵ�λ��
			//drawChessboardCorners(view_gray, boardSize, imgPointsBuf, true);
			//imshow("Camera Calibration", view_gray);
			//waitKey(500);
		}
	}
	cout << "�ǵ���ȡ���" << endl;

// ����궨##############################################################
	cout << "��ʼ����궨.........\n";

	Size squareSize = Size(SQUARESIZEX, SQUARESIZEY); // ÿ�����̸����ʵ��С
	vector<vector<Point3f>> objectPoints; // ����ǵ����ά����
	vector<int> pointCounts;  // ÿ��ͼ���нǵ������
	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0)); // �������5������ϵ����k1,k2,p1,p2,k3
	vector<Mat> rvecsMat;  // �洢ÿ��ͼ�����ת����
	vector<Mat> tvecsMat;  // �洢ÿ��ͼ���ƽ������ 

	// ��ʼ���궨���Ͻǵ�����꣬�������еĽǵ㶼����Z=0��ƽ���ϣ������̿����ϵĵ�һ���ǵ�Ϊ��0��0��0��
	int i, j, t;
	for (t = 0; t < imgCount; t++)
	{
		vector<Point3f> tempPointSet;
		for (i = 0; i < boardSize.height; i++)
		{
			for (j = 0; j < boardSize.width; j++)
			{
				Point3f realPoint;
				// ����궨�������������ϵ��z=0��ƽ���� 
				realPoint.x = i * squareSize.width;
				realPoint.y = j * squareSize.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		objectPoints.push_back(tempPointSet);
		// ��ʼ��ÿ��ͼ���еĽǵ��������ٶ�ÿ��ͼ���ж����Կ��������ı궨��
		pointCounts.push_back(boardSize.width*boardSize.height);
	}
	// ��calibrateCamera()����������ڲ�����ÿ��ͼƬ�����ԭ���ƽ�ƾ������ת����
	calibrateCamera(objectPoints, imgPointsSeq, imgSize, K, distCoeffs, rvecsMat, tvecsMat);
	cout << "�ڲ�����K:\n " << K << endl;


//�Ա궨�����������#######################################
	cout << "��ʼ���۱궨���������������\n";
	double total_err = 0.0; // ����ͼ���ƽ�������ܺ�
	double err = 0.0; // ÿ��ͼ���ƽ�����
	vector<Point2f> image_points2; // �������ڲ��������¼���õ���ͶӰ��
	cout << "\tÿ��ͼ��ı궨��\n";
	for (i = 0; i < imgCount; i++)
	{
		vector<Point3f> tempPointSet = objectPoints[i];
		// ͨ���õ����������������������̵Ľǵ��������ͶӰ���㣬���浽image_points2��
		projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], K, distCoeffs, image_points2);
		
		// �����µ�ͶӰ��;ɵ�ͶӰ��֮������
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
		cout << "��" << i + 1 << "��ͼ���ƽ����" << err << "����" << endl;
	}
	cout << "����ƽ����" << total_err / imgCount << "����" << endl;
	cout << "�������\n";
	cout << "finish getIntrinsicMat function!\n\n\n";
}

