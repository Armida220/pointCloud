#pragma once

// myCalibration
#define INPUT_SETTINGS_FILE "E:\\Source\\repos\\PointCloud\\PointCloud\\input.xml" // 
#define CALIBRATION_IMG_DIR "E:\\captures\\0225\\CAP3\\" // 标定用的图像的存储目录
#define CALIBRATION_CATALOG_FILE "E:\\captures\\0225\\CAP3\\calibdata.txt" // 记录标定用的图像的文本文件
//#define BOARDSIZEX 5 // 棋盘每行角点数量
//#define BOARDSIZEY 8 // 棋盘每列角点数量
//#define SQUARESIZE 21.5 // 棋盘每格的宽,单位mm
#define BOARDSIZEX 9 // 棋盘每行数量
#define BOARDSIZEY 12 // 棋盘每列角点数量
#define SQUARESIZE 30 // 棋盘每格的宽,单位mm


#define IMAGEWIDTH 2592
#define IMAGEHEIGHT 1944

// BinocularVision
#define CONTRIBUTE_IMG_DIR "../photo" // 重建用图像的目录
#define LCameraIMG "../photo/r2.jpg" // LCamera拍摄的图片
#define RCameraIMG "../photo/r1.jpg" // RCamera拍摄的图片
#define RANSACREPROJTHRESHOLD 3 // RANSAC拒绝阈值
#define REBUILDNUM 3 // 3D重建点的数量