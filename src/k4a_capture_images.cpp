#include <stdio.h>
#include "camera_k4a.hpp"

#include <k4a/k4a.hpp>
#include <opencv2/opencv.hpp>
#include <string>

int main()
{
    K4a k4a_device;
 

    k4a_device.capture_images("/home/li/camera_ws/src/camera_bridge/workspace/images","R2"); // 手动拍摄


    return 0;
}