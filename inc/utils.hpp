#pragma once
#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "block_recognizer.hpp"

// 定义平面滤波的关键参数（可根据实际场景调整）
#define DEPTH_DIFF_THRESHOLD 0.05f // 平面内相邻像素的最大深度差（米，越小滤波越严）
#define MAX_DEPTH 3.0f             // 最大有效深度
#define MIN_DEPTH 0.01f            // 最小有效深度（避免0值和极浅噪声）

struct BoundingBox3D
{
    cv::Point3f min_pt;      // 边界最小点
    cv::Point3f max_pt;      // 边界最大点
    cv::Point3f center;      // 目标中心点，x左右均值，y上下均值，z前方距离
    cv::Vec3f principal_dir; // 目标偏角，yaw(水平偏角),pitch(垂直偏角)，弧度制
    int cls_ID;              
    std::string cls_name;
};
inline float Cal_Dist(float x, float y, float z)
{
    return sqrt(x * x + y * y + z * z);
}