// BinocularVision.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "myCalibration.h"
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

using namespace cv;
using namespace std;

/*
@function 实现了像素坐标(qx,qy)到图像坐标(px,py)的转换
@param q 点的像素坐标
@param K 相机的内部矩阵
*/
Point2d pixel2cam(const Point2d& q, const Mat& K)
{
	return Point2d
	(
		(q.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
		(q.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
	);
}


/*
@function 三维重建by三角测量
@param K 相机内部矩阵(假设左右相机内部矩阵相同)
@param R 右相机相对左相机的旋转矩阵
@param T 右相机相对左相机的平移矩阵
@param p1 代求点在图1的像素坐标
@param p2 代求点在图2的像素坐标
@param structure 

@note 此函数假设了左相机的光心同世界坐标系的原点重合
*/
void reconstruct(const Mat& K, const Mat& R, const Mat& T, const vector<Point2f>& p1, const vector<Point2f>& p2, Mat& structure)
{
	//两个相机的投影矩阵
	Mat proj1(3, 4, CV_32FC1);
	Mat proj2(3, 4, CV_32FC1);

	proj1(Range(0, 3), Range(0, 3)) = Mat::eye(3, 3, CV_32FC1);
	proj1.col(3) = Mat::zeros(3, 1, CV_32FC1);
	/*
	       | 1 0 0 0 |
	proj1= | 0 1 0 0 |
	       | 0 0 1 0 |
	*/
	R.convertTo(proj2(Range(0, 3), Range(0, 3)), CV_32FC1);
	T.convertTo(proj2.col(3), CV_32FC1);

	Mat fK;
	K.convertTo(fK, CV_32FC1);
	proj1 = fK * proj1;
	proj2 = fK * proj2;

	// 用triangulatePoints()完成三角测量重建点，得到的3*4齐次坐标存储到structure中
	triangulatePoints(proj1, proj2, p1, p2, structure);
}


/*
@function 通过opencv的函数接口求出了所需的各个矩阵并完成了三维重建
@param src1 左相机的图片
@param src2 右相机的图片
@param K 相机内部矩阵 
@param E 输出的3*3本质矩阵
@param F 输出的3*3基本矩阵
@param H 输出的3*3单应性矩阵
@param R 输出的3*3旋转矩阵
@param T 输出的1*3平移矩阵

@note 求得的结果很不精确，且又可能出现Z<0的奇葩bug
*/
void getMats(const Mat src1, const Mat src2, const Mat K, Mat& E, Mat& F, Mat& H, Mat& R, Mat& T)
{
	cout << "running getEssentialMat()\n";

	if (src1.data == NULL || src2.data == NULL)
	{
		cout << "no image data" << endl;
		exit(1);
	}

// detect keypoints by SIFT
	Ptr<Feature2D> sift = xfeatures2d::SIFT::create(500); 
	vector<KeyPoint> keypoints1, keypoints2; 
	Mat descriptor1, descriptor2;
	// 检测出图像中的关键点
	sift->detectAndCompute(src1, Mat(), keypoints1, descriptor1);
	sift->detectAndCompute(src2, Mat(), keypoints2, descriptor2);
	// 绘制关键点
	//drawKeypoints(src1, keypoints1, src1);
	//drawKeypoints(src2, keypoints2, src2);
	//imshow("SIFT keypoints in src1", src1);
	//imshow("SIFT keypoints in src2", src2);

// match keypoints by BF
	vector<DMatch> matches;
	Ptr<DescriptorMatcher> descriptor_matcher = BFMatcher::create(4, true);
	// 对特征描述符做BF匹配
	descriptor_matcher->match(descriptor1, descriptor2, matches);
	// 绘制匹配点
	Mat match_img;
	drawMatches(src1, keypoints1, src2, keypoints2, matches, match_img);
	imshow("过滤前src1&src2匹配图", match_img);

// 计算单应性矩阵,基础矩阵,本质矩阵
	// 保存匹配对序号
	vector<int> queryIdxs(matches.size()), trainIdxs(matches.size());
	for (size_t i = 0; i < matches.size(); i++)
	{
		queryIdxs[i] = matches[i].queryIdx;
		trainIdxs[i] = matches[i].trainIdx;
	}
	// keyPoint 转为 point数组
	vector<Point2f> points1;
	KeyPoint::convert(keypoints1, points1, queryIdxs);
	vector<Point2f> points2;
	KeyPoint::convert(keypoints2, points2, trainIdxs);
	// 计算单应性矩阵
	H = findHomography(Mat(points1), Mat(points2), FM_RANSAC, RANSACREPROJTHRESHOLD);
	cout << "单应矩阵H:" << endl << H << endl;;
	// 计算基础矩阵
	F = findFundamentalMat(Mat(points1), Mat(points2), FM_RANSAC, RANSACREPROJTHRESHOLD);
	cout << "基础矩阵F:" << endl << F << endl;;
	// 计算本质矩阵
	E = findEssentialMat(Mat(points1), Mat(points2), K, FM_RANSAC, 0.999, RANSACREPROJTHRESHOLD);
	cout << "本质矩阵E:" << endl << E << endl;


// 绘制RANSAC筛选后的匹配图像
	vector<char> matchesMask(matches.size(), 0);
	Mat points1t;
	// 利用单应矩阵计算points1中的点在src2中的位置
	perspectiveTransform(Mat(points1), points1t, H);
	for (size_t i1 = 0; i1 < points1.size(); i1++)  //保存‘内点’
	{
		if (norm(points2[i1] - points1t.at<Point2f>((int)i1, 0)) <= RANSACREPROJTHRESHOLD) //给内点做标记
		{
			matchesMask[i1] = 1;
		}
	}
	Mat match_img2;   //滤除‘外点’后
	drawMatches(src1, keypoints1, src2, keypoints2, matches, match_img2, Scalar(0, 0, 255), Scalar::all(-1), matchesMask);
	imshow("过滤后面src1&src2匹配图", match_img2);

// 分解本质矩阵E得到R，T
	int pass_count = recoverPose(E, points1, points2, K, R, T);
	cout << "旋转矩阵R:\n" << R << endl;
	cout << "平移矩阵T:\n" << T << endl;

// 验证E=t^R
	Mat S = (Mat_<double>(3, 3) << 0, -T.at<double>(2, 0), T.at<double>(1, 0), T.at<double>(2, 0), 0, -T.at<double>(0, 0),- T.at<double>(1, 0), T.at<double>(0, 0), 0);
	cout << "T^R =" << endl << S * R << endl;

//验证对极约束 q2Fq1 = 0 | p2(Tt^R)p1 = 0 | p2Ep1 = 0	
	cout << "验证极线约束：q2Fq1 = 0 \t p2RSp1 = 0 \t p2Ep1 = 0\n";
	int maskIdx = 0;
	for (DMatch m : matches)
	{
		Point2d pt1 = pixel2cam(keypoints1[m.queryIdx].pt, K);
		Point2d pt2 = pixel2cam(keypoints2[m.trainIdx].pt, K);
		Mat y1 = (Mat_<double>(3, 1) << pt1.x, pt1.y, 1);
		Mat y2 = (Mat_<double>(3, 1) << pt2.x, pt2.y, 1);
		Mat z1 = (Mat_<double>(3, 1) << keypoints1[m.queryIdx].pt.x, keypoints1[m.queryIdx].pt.y, 1);
		Mat z2 = (Mat_<double>(3, 1) << keypoints2[m.trainIdx].pt.x, keypoints2[m.trainIdx].pt.y, 1);

		Mat useRS = y2.t()*R*S*y1;
		Mat useE = y2.t()*E*y1;
		Mat useF = z2.t()*F*z1;

		cout << "epipolar constraint: " << "mask = " << (matchesMask[++maskIdx] == 1) << "\t" << useF << "\t"<< useE << "\t" << useRS << endl;
	}
	cout << "\n\n\n";

// 三维重建(从匹配点中选择REBUILDNUM个点进行三角重建)
	Mat structure;
	vector<Point2f> ps1, ps2; // 待重建的点数组
	int pCount=0;
	for (int i = 1; i <= matches.size(); i++)
	{
		int index1 = matches[i].queryIdx;
		int index2 = matches[i].trainIdx;
		if (matchesMask[i] != 1)
		{
			continue;
		}
		if (++pCount > REBUILDNUM)
		{
			break;
		}
		// cout << index1 <<","<< index2 << endl;
		ps1.push_back(keypoints1[index1].pt);
		ps2.push_back(keypoints2[index2].pt);
		cout << "重建点" << pCount << "在src1的像素坐标( x:" << keypoints1[index1].pt.x << ", " << "y:" << keypoints1[index1].pt.y << " )" << endl;
		cout << "重建点" << pCount << "在src2的像素坐标( x:" << keypoints2[index2].pt.x << ", " << "y:" << keypoints2[index2].pt.y << " )" << endl;
	}
	// 绘制重建点
	for (int i=0; i < ps1.size(); i++)
	{
		circle(src1, ps1[i], 5, cv::Scalar(0, 0, 255));
		circle(src2, ps2[i], 5, cv::Scalar(0, 0, 255));
	}
	imshow("重建点在src1中", src1);
	imshow("重建点在src2中", src2);

	// 计算3D坐标
	reconstruct(K, R, T, ps1, ps2, structure);
	// 打印三维坐标

	cout << structure.size();
	cout << "\n三维坐标\n";
	for (int i = 0; i < structure.size().width * 4; )
	{
		cout << structure.at<float>(i)   / structure.at<float>(i + 3) << ", ";
		cout << structure.at<float>(i+1) / structure.at<float>(i + 3) << ", ";
		cout << structure.at<float>(i+2) / structure.at<float>(i + 3) << "\n";
		i += 4;
	}
}

int main()
{
	// 计算相机内部矩阵
	Mat K = Mat(3, 3, CV_32FC1, Scalar::all(0)); 
	myCalibration::getIntrinsicMat(CALIBRATION_CATALOG_FILE, K);
	// 载入双目相机的图片
	Mat src1 = imread(LCameraIMG);
	Mat src2 = imread(RCameraIMG);
	Mat E, F, H, R, T; 
	getMats(src1, src2, K, E, F, H, R, T);
	
	// 交换输入顺序应得到的 T1+T2 = (0,0,0)，但实际误差有点
	// 大，例如[-0.3986413985876479;0.1962793652866788;0.1386065552667751]
	//Mat E1, F1, H1, R1, T1;
	//getMats(src2, src1, K, E1, F1, H1, R1, T1);
	//cout << "T1:" << T;
	//cout << "\nT2:" << T1;
	//cout << "\nT1+T2:\n" << T + T1;


	waitKey(0);
}
