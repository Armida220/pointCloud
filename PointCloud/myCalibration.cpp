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
@param infile Ŀ¼�ļ���·��,���б�����������������궨���ļ���
@param K      ��������ڲ�����
*/
void myCalibration::getIntrinsicMat(std::string infile, cv::Mat& K, cv::Mat& distCoeffs)
{
	cout << "��ʼ���ǵ�................\n";

	ifstream fin(infile); 
	int imgCount = 0; // �궨��ͼ�������
	Size imgSize; // ͼ��ߴ�(Ĭ������ͼ��ߴ�һ�£����ɵ�һ��ͼ��õ�)
	Size boardSize = Size(BOARDSIZEX-1, BOARDSIZEY-1); // ����ÿ��ÿ�еĽǵ���
	vector<Point2f> imgPointsBuf; // ����һ��ͼ���ϼ�⵽�Ľǵ�
	float squareSize = SQUARESIZE; // ÿ�����̸����ʵ��С
	vector<vector<Point3f>> objectPoints(1); // ����ǵ����ά����
	vector<vector<Point2f>> imagePoints;  // ����ǵ�Ķ�ά����

	vector<Mat> rvecsMat;  // �洢����ͼ�����ת����
	vector<Mat> tvecsMat;  // �洢����ͼ���ƽ������


// ----------------------��ȡ�ǵ�----------------------------	
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
		if (!findChessboardCorners(imgInput, boardSize, imgPointsBuf, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
			+ CALIB_CB_FAST_CHECK))
		{
			cout << filename <<"�м�ⲻ���ǵ�\n";
			exit(1);
		}
		else
		{
			Mat view_gray;
			cvtColor(imgInput, view_gray, COLOR_RGB2GRAY);
			
			cout << imgPointsBuf;
			// �����ؾ�ȷ��
			//find4QuadCornerSubpix(view_gray, imgPointsBuf, Size(11, 11)); 
//			TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
			cornerSubPix(view_gray, imgPointsBuf, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));


			imagePoints.push_back(imgPointsBuf);  //���������ؽǵ�
			// ��ʾ�ǵ�λ��
			//drawChessboardCorners(view_gray, boardSize, imgPointsBuf, true);
			//imshow("Camera Calibration", view_gray);
			//waitKey(500);
		}
	}

	cout << "�ǵ���ȡ���!\n";
	cout << "��ʼ����궨.........\n";
// -------------------------------------------------
	// ��ʼ���궨���Ͻǵ������
	calcBoardCornerPositions(boardSize, squareSize, objectPoints[0]);
	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	// ��calibrateCamera()����������ڲ�����ÿ��ͼƬ�����ԭ���ƽ�ƾ������ת����
	calibrateCamera(objectPoints, imagePoints, imgSize, K, distCoeffs, rvecsMat, tvecsMat);
	

	cout << "\tK:\n\t" << K << endl;
	cout << "\tdistCoeffs:\n\t" << distCoeffs << endl;

	// ������ͶӰ���
	vector<float> reprojErrs;
	float totalAvgErr = 0;
	computeReprojectionErrors(imgCount, objectPoints,
		rvecsMat, tvecsMat, K, distCoeffs, 
		imagePoints, reprojErrs, &totalAvgErr);



	cout << "�궨����\n";
	cout << "����궨���\n";
	// ���浽�ļ�
	saveToFile("./result_1.yml", imgSize, boardSize, squareSize,
		K, distCoeffs,
		rvecsMat, tvecsMat,
		reprojErrs,
		imagePoints,
		&totalAvgErr);
	cout << "�������\n";

}

void  myCalibration::calcBoardCornerPositions(Size boardSize, float squareSize,
	vector<Point3f>& corners
)
{
	corners.clear();

	// ��ʼ���궨���Ͻǵ�����꣬�������еĽǵ㶼����Z=0��ƽ���ϣ������̿����ϵĵ�һ���ǵ�Ϊ��0��0��0��
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
	cout << "\t��ʼ���۱궨���������������\n";
	size_t totalPoints = 0;
	float err = 0.0;       
	vector<Point2f> image_points2; // �������ڲ��������¼���õ���ͶӰ��
	cout << "\t\tÿ��ͼ��ı궨��\n";
	for (int i = 0; i < imgCount; i++)
	{
		// ͨ���õ����������������������̵Ľǵ��������ͶӰ���㣬���浽image_points2��
		projectPoints(objectPoints[i], rvecsMat[i], tvecsMat[i], K, distCoeffs, image_points2);

		// ������ͶӰ���ͶӰ��֮������
		err = norm(imagePoints[i], image_points2, NORM_L2);

		size_t n = objectPoints[i].size();
		float perViewErr = (float)std::sqrt(err*err / n);
		cout << "\t\t\t��" << i + 1 << "��ͼ���ƽ����" << perViewErr << "����" << endl;
		reprojErrs.push_back(perViewErr);
		*totalErrs += err * err;
		totalPoints += n;
		//totalErr += err /= objectPoints[i].size();
		//cout << "\t\t\t��" << i + 1 << "��ͼ���ƽ����" << err << "����" << endl;
	}

	*totalErrs = sqrt(*totalErrs / totalPoints);
	cout << "\t\t����ƽ����" << *totalErrs << "����" << endl;
	
	
	cout << "\t�������\n";
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

