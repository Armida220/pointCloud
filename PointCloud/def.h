#pragma once

// myCalibration
#define INPUT_SETTINGS_FILE "E:\\Source\\repos\\PointCloud\\PointCloud\\input.xml" // 
#define CALIBRATION_IMG_DIR "E:\\captures\\0225\\CAP3\\" // �궨�õ�ͼ��Ĵ洢Ŀ¼
#define CALIBRATION_CATALOG_FILE "E:\\captures\\0225\\CAP3\\calibdata.txt" // ��¼�궨�õ�ͼ����ı��ļ�
//#define BOARDSIZEX 5 // ����ÿ�нǵ�����
//#define BOARDSIZEY 8 // ����ÿ�нǵ�����
//#define SQUARESIZE 21.5 // ����ÿ��Ŀ�,��λmm
#define BOARDSIZEX 9 // ����ÿ������
#define BOARDSIZEY 12 // ����ÿ�нǵ�����
#define SQUARESIZE 30 // ����ÿ��Ŀ�,��λmm


#define IMAGEWIDTH 2592
#define IMAGEHEIGHT 1944

// BinocularVision
#define CONTRIBUTE_IMG_DIR "../photo" // �ؽ���ͼ���Ŀ¼
#define LCameraIMG "../photo/r2.jpg" // LCamera�����ͼƬ
#define RCameraIMG "../photo/r1.jpg" // RCamera�����ͼƬ
#define RANSACREPROJTHRESHOLD 3 // RANSAC�ܾ���ֵ
#define REBUILDNUM 3 // 3D�ؽ��������