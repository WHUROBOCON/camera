#include <iostream>

#include <opencv2/opencv.hpp>

#include "camera_k4a.hpp"
#include "myinfer.hpp"
#include <main.hpp>
#include <csignal>

// 条件编译ROS支持
#ifdef BUILD_WITH_ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

#endif
volatile sig_atomic_t stop_flag = 0;

void sigintHandler(int sig)
{
    stop_flag = 1;
}
int main(
#ifdef BUILD_WITH_ROS
    int argc, char **argv
#endif
)
{
#ifdef BUILD_WITH_ROS
    // ROS初始化
    ros::init(argc, argv, "k4a_detector");
    ros::NodeHandle nh;
    signal(SIGINT, sigintHandler); // 捕获 Ctrl+C

    // 创建发布者，发布一个 topic：class_id + center xyz
    ros::Publisher target_pub = nh.advertise<std_msgs::String>("/k4a/target_info", 10);

    ROS_INFO("K4A Detector started with ROS support");
#else
    std::cout << "[INFO] K4A Detector started (non-ROS mode)" << std::endl;
#endif

    try
    {
        K4a k4a_device; // 类里是构造函数，自动调用configuration函数
        // k4a_device.Configuration(); //不要有这一句，会报错
        Yolo yolo; // 实例化类
        std::string engine_path = "/home/li/camera_ws/src/camera_bridge/workspace/model_generate/yolo_dete_full2/weights/best.engine";
        yolo::BoxArray detections;

        bool first_cloud = true;                                                       // 第一次生成点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // 定义点云指针
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PointCloud Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addCoordinateSystem(0.2);
        yolo.Yolov8_Enable(engine_path); // 检测引擎适用于yolov8s.engine
        // yolo.Yolov8_Seg_Enable(engine_path); // 分割引擎适用于best.engine

        while (!stop_flag)
        {
            cv::Mat color_image, depth_image;
            cv::Mat gray;
            cv::Mat gray3;
            cv::Mat depth_display;

            k4a_device.Image_to_Cv(color_image, depth_image); // 获取彩色和深度图像

            cv::cvtColor(color_image, gray, cv::COLOR_BGR2GRAY);
            cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
            clahe->setClipLimit(2.0);
            clahe->apply(gray, gray);
            cv::cvtColor(gray, gray3, cv::COLOR_GRAY2BGR);

            yolo.Single_Inference(gray3, detections);

            static int frame_id = 0;

            BoundingBox3D bbox = k4a_device.Value_Mask_to_Pcl(cloud, depth_image, detections);
            frame_id++;
            k4a_device.Color_With_Mask(color_image, detections); // 显示带有检测结果的彩色图像

            depth_image.convertTo(depth_display, CV_8U, 255.0 / 4000.0);
            k4a_device.Depth_With_Mask(depth_display, detections); // 用显示图，不用原始 CV_16U
            if (frame_id % 5 == 0)
            {
                std::cout << "Class_ID: " << bbox.cls_ID
                          << " Center: [" << bbox.center.x << ", "
                          << bbox.center.y << ", "
                          << bbox.center.z << "]\n";
            }
#ifdef BUILD_WITH_ROS
            // ---- 发布 ROS 简化后的消息 ----
            std_msgs::String msg;
            std::stringstream ss;

            ss << bbox.cls_ID << ","
               << bbox.center.x << ","
               << bbox.center.y << ","
               << bbox.center.z;

            msg.data = ss.str();
            target_pub.publish(msg);
#endif
            if (!color_image.empty())
                cv::imshow("Detect Color Image", color_image); // 显示彩色图像
            if (!depth_display.empty())
            {
                cv::imshow("Depth Image", depth_display);
            }

            //  更新 PCL 可视化器
            if (first_cloud)
            {
                viewer->addPointCloud<pcl::PointXYZ>(cloud, "target_cloud");
                viewer->resetCameraViewpoint("target_cloud"); // 自动对准目标点云
                first_cloud = false;
            }
            else
            {
                viewer->updatePointCloud<pcl::PointXYZ>(cloud, "target_cloud");
            }

            char key = (char)cv::waitKey(10);
            if (key == 'q' || key == 27) // 按 q 或 Esc 键退出
                break;
            viewer->spinOnce(10); // 更新显示
#ifdef BUILD_WITH_ROS
            // ROS循环控制
            ros::spinOnce();
#endif
        }
        return EXIT_SUCCESS;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}
