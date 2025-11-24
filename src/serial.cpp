#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <sstream>
#include <vector>
#include <iomanip>
#include <cstring>

// 串口对象
serial::Serial ser;

// 回调函数
void targetCallback(const std_msgs::String::ConstPtr& msg)
{
    // 假设 msg->data 格式为 "cls_ID x y z"
    std::istringstream ss(msg->data);
    int cls_ID;
    float x, y, z;
    ss >> cls_ID >> x >> y >> z;

    // 构造发送帧
    std::vector<uint8_t> frame;
    frame.push_back(0xFF);
    frame.push_back(0xFE);
    frame.push_back(0x01); // 数据类型标识

    auto append_float = [&](float value) {
        uint8_t bytes[sizeof(float)];
        std::memcpy(bytes, &value, sizeof(float));
        for (size_t i = 0; i < sizeof(float); ++i)
            frame.push_back(bytes[i]);
    };

    frame.push_back(static_cast<uint8_t>(cls_ID));
    append_float(x);
    append_float(y);
    append_float(z);

    frame.push_back(0xAA);
    frame.push_back(0xDD);

    // 发送
    if (ser.isOpen()) {
        ser.flushOutput();
        ser.write(frame);
    }

    // 打印调试信息
    std::ostringstream oss;
    oss << "Sent frame (" << frame.size() << " bytes): ";
    for (auto b : frame) {
        oss << std::uppercase << std::setfill('0') << std::setw(2) 
            << std::hex << static_cast<int>(b) << " ";
    }
    ROS_INFO_STREAM(oss.str());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "k4a_serial_node");
    ros::NodeHandle nh;

    // 初始化串口
    try {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port: " << e.what());
        return -1;
    }

    if (!ser.isOpen()) {
        ROS_ERROR("Failed to open serial port.");
        return -1;
    }
    ROS_INFO("Serial port initialized.");

    // 订阅 /k4a/target_info
    ros::Subscriber sub = nh.subscribe("/k4a/target_info", 10, targetCallback);

    ros::spin();

    ser.close();
    return 0;
}
