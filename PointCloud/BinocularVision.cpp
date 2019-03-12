#include <io.h>
#include "myCalibration.h"
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

using namespace cv;
using namespace std;



vector<string> img_names;
/*

*/
void getFiles(string path, vector<string>& files)
{
	intptr_t    hFile = 0;
	struct _finddata_t fileinfo;
	string p;

	if ((hFile = _findfirst(p.assign(path).append("\\*.jpg").c_str(), &fileinfo)) != -1) {
		do {
			if ((fileinfo.attrib &  _A_SUBDIR)) {
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
				{
					files.push_back(p.assign(path).append("\\").append(fileinfo.name));
					getFiles(p.assign(path).append("\\").append(fileinfo.name), files);
				}
			}
			else {
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}

		} while (_findnext(hFile, &fileinfo) == 0);

		_findclose(hFile);
	}
}


void extract_features(
	vector<string>& image_names,
	vector<vector<KeyPoint>>& key_points_for_all,
	vector<Mat>& descriptor_for_all,
	vector <vector<Vec3b>>& colors_for_all
)
{
	key_points_for_all.clear();
	descriptor_for_all.clear();
	Mat image;

	// 读取图像，获取图像特征点并保存
	Ptr<Feature2D> sift = xfeatures2d::SIFT::create();
	for(vector<string>::iterator it = image_names.begin(); it != image_names.end(); ++it)
	{
		image = imread(*it);
		if(image.empty()) 
			continue;
		cout << "Extracting features: " << *it << endl;

		vector<KeyPoint> key_points;
		Mat descriptor;
		// 偶尔出现内存分配失败的错误  Detects keypoints and computes the descriptors
		// sift->detectAndCompute(image, noArray(), key_points, descriptor);
		sift->detect(image, key_points);
		sift->compute(image, key_points, descriptor);

		// 特征点过少，则排除该图像
		if (key_points.size() <= 10)
			continue;
		
		key_points_for_all.push_back(key_points);
		descriptor_for_all.push_back(descriptor);

		// 三通道 存放该位置三通道颜色
		//vector<Vec3b> colors(key_points.size());	
		//for (int i = 0; i < key_points.size(); ++i)
		//{
		//	Point2f& p = key_points[i].pt;
		//	if (p.x <= image.rows && p.y <= image.cols)
		//		colors[i] = image.at<Vec3b>(p.x, p.y);
		//}

		//colors_for_all.push_back(colors);
	}
}


void match_features(Mat& query, Mat& train, vector<DMatch>& matches)
{
	vector<vector<DMatch>> knn_matches;
	BFMatcher matcher(NORM_L2);
	matcher.knnMatch(query, train, knn_matches, 2);

	// 获取满足Ratio Test的最小匹配的距离
	float min_dist = FLT_MAX;
	for (int r = 0; r < knn_matches.size(); ++r)
	{
		// Rotio Test
		if (knn_matches[r][0].distance > 0.6 * knn_matches[r][1].distance)
		{
			continue;
		}

		float dist = knn_matches[r][0].distance;
		if (dist < min_dist)
		{
			min_dist = dist;
		}
	}

	matches.clear();
	for (size_t r = 0; r < knn_matches.size(); ++r)
	{
		// 排除不满足Ratio Test的点和匹配距离过大的点
		if (
			knn_matches[r][0].distance > 0.6 * knn_matches[r][1].distance ||
			knn_matches[r][0].distance > 5 * max(min_dist, 10.0f)
			)
		{
			continue;
		}

		// 保存匹配点
		matches.push_back(knn_matches[r][0]);
	}
}


void match_features(vector<Mat>& descriptor_for_all, vector<vector<DMatch>>& matches_for_all)
{
	matches_for_all.clear();
	// n个图像，两两顺次有 n-1 对匹配
	// 1与2匹配，2与3匹配，3与4匹配，以此类推
	for (int i = 0; i < descriptor_for_all.size() - 1; ++i)
	{
		cout << "Matching images " << i << " - " << i + 1 << endl;
		vector<DMatch> matches;
		match_features(descriptor_for_all[i], descriptor_for_all[i + 1], matches);
		matches_for_all.push_back(matches);
	}
}

bool find_transform(Mat& K, vector<Point2f>& p1, vector<Point2f>& p2, Mat& R, Mat& T, Mat& mask)
{
	// 根据匹配点求取本征矩阵，使用RANSAC，进一步排除失配点
	Mat E = findEssentialMat(p1, p2, K, RANSAC, 0.999, 1.0, mask);
	if (E.empty())
		return false;

	// 对于RANSAC而言，outlier数量大于50%时，结果是不可靠的
	double feasible_count = countNonZero(mask);
	if (feasible_count <= 15 || (feasible_count / p1.size()) < 0.6)
		return false;

	// 分解本征矩阵，获取相对变换  Returns the number of inliers which pass the check.
	int pass_count = recoverPose(E, p1, p2, K, R, T, mask);

	// 同时位于两个相机前方的点的数量要足够大
	if (((double)pass_count) / feasible_count < 0.7)
		return false;
	
	return true;
}



void maskout_points(vector<Point2f>& p1, Mat& mask)
{
	vector<Point2f> p1_copy = p1;
	p1.clear();

	for (int i = 0; i < mask.rows; ++i)
	{
		if (mask.at<uchar>(i) > 0)
		{
			p1.push_back(p1_copy[i]);
		}
	}
}


void reconstruct(Mat& K, Mat& R1, Mat& T1, Mat& R2, Mat& T2, vector<Point2f>& p1, vector<Point2f>& p2, vector<Point3f>& structure)
{
	// 两个相机的投影矩阵[R, T], triangulatePoints只支持float型
	Mat proj1(3, 4, CV_32FC1);
	Mat proj2(3, 4, CV_32FC1);

	R1.convertTo(proj1(Range(0, 3), Range(0, 3)), CV_32FC1);
	//T1.convertTo(proj2(Range(0, 3), Range(3, 4)), CV_32FC1);
	T1.convertTo(proj1.col(3), CV_32FC1);

	R2.convertTo(proj2(Range(0, 3), Range(0, 3)), CV_32FC1);
	//T2.convertTo(proj2(Range(0, 3), Range(3, 4)), CV_32FC1);
	T2.convertTo(proj2.col(3), CV_32FC1);

	Mat fK;
	K.convertTo(fK, CV_32FC1);
	proj1 = fK * proj1;
	proj2 = fK * proj2;

	// 三角重建
	Mat s;
	triangulatePoints(proj1, proj2, p1, p2, s);

	structure.clear();
	structure.reserve(s.cols); // 扩容
	for (int i = 0; i < s.cols; ++i)
	{
		Mat_<float> col = s.col(i);
		col /= col(3);	// 齐次坐标
		structure.push_back(Point3f(col(0), col(1), col(2)));
	}
}



void init_structure(
	Mat K,
	vector<vector<KeyPoint>>& key_points_for_all,
	vector<vector<Vec3b>>& colors_for_all,
	vector<vector<DMatch>>& matches_for_all,
	vector<Point3f>& structure,
	vector<vector<int>>& correspond_struct_idx,
	vector<Vec3b>& colors,
	vector<Mat>& rotations,
	vector<Mat>& motions
)
{
	// 计算头两幅图像之间的变换矩阵
	vector<Point2f> p1, p2;
	vector<Vec3b> c2;
	Mat R, T;	// 旋转矩阵和平移向量
	Mat mask;	// mask中大于零的点代表匹配点，等于零的点代表失配点
	//get_matched_points(key_points_for_all[0], key_points_for_all[1], matches_for_all[0], p1, p2);
	p1.clear();
	p2.clear();
	for (int i = 0; i < matches_for_all[0].size(); ++i)
	{
		p1.push_back(key_points_for_all[0][matches_for_all[0][i].queryIdx].pt);
		p2.push_back(key_points_for_all[1][matches_for_all[0][i].trainIdx].pt);
	}
	//get_matched_colors(colors_for_all[0], colors_for_all[1], matches_for_all[0], colors, c2);
	
	find_transform(K, p1, p2, R, T, mask);	// 三角分解得到R， T 矩阵

	// 对头两幅图像进行三维重建
	maskout_points(p1, mask);
	maskout_points(p2, mask);
	//maskout_colors(colors, mask);

	Mat R0 = Mat::eye(3, 3, CV_64FC1);
	Mat T0 = Mat::zeros(3, 1, CV_64FC1);
	reconstruct(K, R0, T0, R, T, p1, p2, structure);	// 三角化
	// 保存变换矩阵
	rotations.push_back(R0);
	rotations.push_back(R);
	motions.push_back(T0);
	motions.push_back(T);

	// 将correspond_struct_idx的大小初始化为与key_points_for_all完全一致
	correspond_struct_idx.clear();
	correspond_struct_idx.resize(key_points_for_all.size());
	for (int i = 0; i < key_points_for_all.size(); ++i)
	{
		correspond_struct_idx[i].resize(key_points_for_all[i].size(), -1);
	}

	// 填写头两幅图像的结构索引
	int idx = 0;
	vector<DMatch>& matches = matches_for_all[0];
	for (int i = 0; i < matches.size(); ++i)
	{
		if (mask.at<uchar>(i) == 0)
			continue;

		correspond_struct_idx[0][matches[i].queryIdx] = idx;	// 如果两个点对应的idx 相等 表明它们是同一特征点 idx 就是structure中对应的空间点坐标索引
		correspond_struct_idx[1][matches[i].trainIdx] = idx;
		++idx;
	}
}


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
void reconstruct(const Mat K, const Mat R, const Mat T, const vector<Point2f> p1, const vector<Point2f> p2, Mat& structure)
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
@function 三维重建(从匹配点中选择REBUILDNUM个点进行三角重建)

*/
void reBuild(const Mat img1, const vector<KeyPoint>& keypoints1,
	const Mat img2, const vector<KeyPoint>& keypoints2,
	const vector<DMatch>& matches, 
	const vector<char>& mask,
	const Mat K, const Mat R, const Mat T, Mat& structure)
{
	vector<Point2f> ps1, ps2; // 待重建的点数组
	int pCount = 0;
//	for (int i = 0; i < matches.size(); i++)
    for (int i = matches.size()-1; i >-1; i--)
	{
		int idx1 = matches[i].queryIdx;
		int idx2 = matches[i].trainIdx;

		if (mask[i] != 1)
		{
			continue;
		}
		if (++pCount > REBUILDNUM)
		{
			break;
		}
		cout << idx1 <<","<< idx2 << endl;
		ps1.push_back(keypoints1[idx1].pt);
		ps2.push_back(keypoints2[idx2].pt);
		cout << "点" << pCount << "在src1的像素坐标( " << keypoints1[idx1].pt.x << ", " << keypoints1[idx1].pt.y << " )\n";
		cout << "点" << pCount << "在src2的像素坐标( " << keypoints2[idx2].pt.x << ", " << keypoints2[idx2].pt.y << " )\n\n";
	}
	// 绘制重建点
	Mat img_matches;
	for (int i = 0; i < ps1.size(); i++)
	{
		circle(img1, ps1[i], 5, Scalar(0, 0, 255));
		circle(img2, ps2[i], 5, Scalar(0, 0, 255));
		//drawMatches(img1, keypoints1, img2, keypoints2, matches, img_matches, Scalar::all(-1), Scalar::all(-1), mask);
	}
	imshow("重建点在src1中", img1);
	imshow("重建点在src2中", img2);
	//imshow("三维重建", img_matches);

	// 计算3D坐标
	reconstruct(K, R, T, ps1, ps2, structure);
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
void getMats(const Mat img1, const Mat img2, const Mat K, Mat& E, Mat& F, Mat& H, Mat& R, Mat& T, Mat& distCoeffs)
{
	cout << "running getMats()\n";

	if (img1.data == NULL || img2.data == NULL)
	{
		cout << "no image data" << endl;
		exit(1);
	}
// 消除镜头畸变
	Mat src1, src2;
	undistort(img1, src1, K, distCoeffs);
	undistort(img2, src2, K, distCoeffs);
    //imshow("校正后的img1", src1);
    //imshow("校正后的img2", src2);


// detect keypoints by SIFT
	Ptr<Feature2D> sift = xfeatures2d::SIFT::create(100);
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
	
	Mat EMask, FMask, HMask;
	// 计算单应性矩阵H
	H = findHomography(Mat(points1), Mat(points2), FM_RANSAC, RANSACREPROJTHRESHOLD, HMask);
	cout << "单应矩阵H:" << endl << H << endl;;
	// 计算基础矩阵F
	F = findFundamentalMat(Mat(points1), Mat(points2), FM_RANSAC, RANSACREPROJTHRESHOLD,0.99, FMask);
	cout << "基础矩阵F:" << endl << F << endl;;
	// 计算本质矩阵E
	E = findEssentialMat(Mat(points1), Mat(points2), K, FM_RANSAC, 0.999, RANSACREPROJTHRESHOLD, EMask);
	cout << "本质矩阵E:" << endl << E << endl;

	// 检测内点的数量
	double feasible_count = countNonZero(EMask);
	cout << (int)feasible_count << "-in-" << points1.size() << endl;
	if (feasible_count <= 15 || (feasible_count / points1.size()) < 0.6)
	{
		cout << "对于RANSAC而言，outlier数量大于50%时，结果是不可靠的";
		//return;
	}


// 验证F和E，K的关系
	Mat K_invert;
	Mat K_invert_transpose;
	invert(K, K_invert);
	transpose(K_invert, K_invert_transpose);
	
	cout << "本质矩阵 K^-1T E K^-1:" << endl << K_invert * E * K_invert_transpose << endl;


// 绘制RANSAC筛选后的匹配图像
	Mat match_img_H, match_img_E, match_img_F;
	drawMatches(src1, keypoints1, src2, keypoints2, matches, match_img_H, Scalar(0, 0, 255), Scalar::all(-1), HMask);
	drawMatches(src1, keypoints1, src2, keypoints2, matches, match_img_E, Scalar(0, 0, 255), Scalar::all(-1), EMask);
	drawMatches(src1, keypoints1, src2, keypoints2, matches, match_img_F, Scalar(0, 0, 255), Scalar::all(-1), FMask);

	imshow("H过滤后的匹配图", match_img_H);
	imshow("E过滤后的匹配图", match_img_E);
	imshow("F过滤后的匹配图", match_img_F);


// 分解本质矩阵E得到R，T
	int pass_count = recoverPose(E, points1, points2, K, R, T, EMask);
	if (((double)pass_count) / feasible_count < 0.7)
	{
		cout << "同时位于两个相机前方的点的数量要足够大";
		//return;
	}
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

		cout << "epipolar constraint: " << "mask = " << (HMask.at<char>(maskIdx++) == 1) << "\t" << useF << "\t"<< useE << "\t" << useRS << endl;
	}
	cout << "\n\n\n";
	
	// 计算三维坐标
	Mat structure;
	reBuild(src1, keypoints1, src2, keypoints2, matches, HMask, K, R, T, structure);
	
	// 打印三维坐标
    cout << structure.size();
	cout << "\n三维坐标\n";
	for (int i = 0; i < structure.size().width * 4; )
	{
		cout << structure.at<float>(i) / structure.at<float>(i + 3) << ", ";
		cout << structure.at<float>(i + 1) / structure.at<float>(i + 3) << ", ";
		cout << structure.at<float>(i + 2) / structure.at<float>(i + 3) << "\n";
		i += 4;
	}
}

vector<vector<KeyPoint>> key_points_for_all;
vector<Mat> descriptor_for_all;
vector<vector<Vec3b>> colors_for_all;
vector<vector<DMatch>> matches_for_all;

vector<Point3f> structure;
vector<vector<int>> correspond_struct_idx;	// 保存第i副图像中第j特征点对应的structure中点的索引
vector<Vec3b> colors;
vector<Mat> rotations;
vector<Mat> motions;


/*
int main()
{

// --------------------------- 相机标定模块 -----------------
    // 计算相机内部矩阵和镜头畸变参数
	Mat K = Mat(3, 3, CV_32FC1, Scalar::all(0)); 
	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));
	myCalibration::getIntrinsicMat(CALIBRATION_CATALOG_FILE, K, distCoeffs);

	
	// 给定的相机内部矩阵
	//Mat K(Matx33d(
	//	3071.477, 0, 1066.067,
	//	0, 3069.834, 1929.784,
	//	0, 0, 1));

// ---------------------------- SFM模块 -----------------------
	// 获取重建用的图像文件名
	//getFiles(CONTRIBUTE_IMG_DIR, img_names);
	//
	//vector<string>::iterator iter;
	//for (iter = img_names.begin(); iter != img_names.end(); iter++)
	//{
	//	cout << *iter << endl;
	//}

	//// 提取所有图像的特征并计算描述子
	//extract_features(img_names, key_points_for_all, descriptor_for_all, colors_for_all);
	//// 对所有图像进行顺次的特征匹配
	//match_features(descriptor_for_all, matches_for_all);

	//// 初始化结构（三维点云）头两幅
	//init_structure(
	//	K,
	//	key_points_for_all,
	//	colors_for_all,
	//	matches_for_all,
	//	structure,
	//	correspond_struct_idx,
	//	colors,
	//	rotations,
	//	motions
	//);

	cout << "successful!!!" << endl;

	waitKey(0);
}
*/
