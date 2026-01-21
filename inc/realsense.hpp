#ifndef REALSENSE_HPP
#define REALSENSE_HPP

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// 点云基础头文件
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

// PCL 滤波头文件
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

// PCL PCA 计算
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <Eigen/Dense>

// 配置文件
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <string>
#include <memory>
#include <unistd.h>

#include "yolo.hpp"
#include "main.hpp"
#include "vision_draw.hpp"

#define COUT_RED_START std::cout << "\033[1;31m";
#define COUT_GREEN_START std::cout << "\033[1;32m";
#define COUT_YELLOW_START std::cout << "\033[1;33m";
#define COUT_BLUE_START std::cout << "\033[1;34m";
#define COUT_PURPLE_START std::cout << "\033[1;35m";
#define COUT_CYAN_START std::cout << "\033[1;36m";
#define COUT_WHITE_START std::cout << "\033[1;37m";
#define COUT_COLOR_END std::cout << "\033[0m";


// 定义相机模式
enum class CameraMode
{
    DEFAULT,      // 彩色 + 深度
    INFRARED_ONLY // 左右红外
};

struct CameraParams
{
    int width, height, fps;// 相机分辨率和帧率
    CameraMode mode = CameraMode::DEFAULT;// 相机模式
    Eigen::Matrix3f rotation;// 3x3 旋转矩阵
    Eigen::Vector3f translation;// 3x1 平移向量.即相机坐标系到机器人坐标系的平移
    float min_dist; // 最小距离阈值
    float max_dist; // 最大距离阈值

   static CameraParams LoadFromFile(const std::string& filepath);
};


class RealSense
{
private:
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::pipeline_profile profile;
    rs2::frameset frameset;
    cv::Mat image_rs_color, image_rs_depth;
    cv::Mat image_rs_infrared_left, image_rs_infrared_right;
    cv::Mat mask;
    int frame_count = 0;
    CameraParams m_params; // 存储当前相机的配置数据

    // 私有构造函数：内部调用 Configuration()
    // 设为私有是为了强制使用 Create_FromFile 或外部显式传参
    RealSense(const CameraParams& params);
public:
    rs2_intrinsics intrinsics_depth;
    rs2_intrinsics intrinsics_color;
    rs2_intrinsics intrinsics_infrared;
    float depth_scale = 0.001f;


    // 工厂函数：普通用，通过路径加载
    static RealSense Create_FromFile(const std::string& config_path);

    // // 配置RealSense对象
    void Configuration();


    // 将摄像头获取的彩色和深度图像转换为OpenCV的cv::Mat格式
    void Image_to_Cv(cv::Mat &image_cv_color, cv::Mat &image_cv_depth);

    // 将获取的彩色图像转换为OpenCV的cv::Mat格式
    void Color_to_Cv(cv::Mat &image_cv_color);

    // 将获取的左右红外图像转换为OpenCV的cv::Mat格式
    void Infrared_to_Cv(cv::Mat &image_cv_infrared_left, cv::Mat &image_cv_infrared_right);

    // 将深度图像转换为PCL的点云格式
    void Value_Depth_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud);

    BoundingBox3D Extract_Object_PointCloud_FromDepth(
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        const cv::Mat &depth_image,
        yolo::BoxArray objs);

    // 保存指定数量的彩色图像
    void Save_Image(int amount, std::string output_dir);

    // 生成目标点云
    void Mask_Depth_to_Pcl(pcl::PointCloud<pcl::PointXYZ> &cloud,
                           const cv::Mat &mask);

    // 抓取图像并转换为cv
    bool Capture(cv::Mat &color, cv::Mat &depth);

    ~RealSense() = default; //析构函数,在对象销毁时自动调用,用于释放资源
};

#endif // REALSENSE_HPP