#pragma once
#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>

inline int getRowFromY(float y, int img_height = 720)
{
    int row = static_cast<int>(y / (img_height / 4.0));
    return std::min(row, 2); // 防止越界
}

inline int getColFromX(float x, int img_width = 1280)
{
    int col = static_cast<int>(x / (img_width / 3.0));
    return std::min(col, 3);
}
struct Pose
{
    double x;     // 单位：米
    double y;     // 单位：米
    double theta; // 朝向角，弧度
};
struct BoundingBox3D
{
    cv::Point3f min_pt;      // 边界最小点
    cv::Point3f max_pt;      // 边界最大点
    cv::Point3f center;      // 目标中心点，x左右均值，y上下均值，z前方距离
    cv::Vec3f principal_dir; // 目标偏角，yaw(水平偏角),pitch(垂直偏角)，弧度制
    std::string cls_name;    // 目标类别名称
    int cls_ID;
};
inline float Cal_Dist(float x, float y, float z)
{
    return sqrt(x * x + y * y + z * z);
}