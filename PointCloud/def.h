#pragma once

// myCalibration
#define CALIBRATION_IMG_DIR "../calibration_img/" // 标定用的图像的存储目录
#define BOARDSIZEX 4 // 棋盘每行角点数量
#define BOARDSIZEY 6 // 棋盘每列角点数量
#define SQUARESIZEX 26 // 棋盘每格的宽,单位mm
#define SQUARESIZEY 26 // 棋盘每格的高,单位mm

// BinocularVision
#define CALIBRATION_CATALOG_FILE "../calibration_img/calibdata.txt" // 记录标定用的图像的文本文件
#define LCameraIMG "../photo/RT3.jpg" // LCamera拍摄的图片
#define RCameraIMG "../photo/RT4.jpg" // RCamera拍摄的图片
#define RANSACREPROJTHRESHOLD 5 // RANSAC拒绝阈值
#define REBUILDNUM 3 // 3D重建点的数量