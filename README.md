# camera_bridge

## 代码运行步骤

1. ROS环境下的编译及运行
    1. 放到ROS的camera_ws/src下
        
        ```
        cd ~/camera_ws/src
        git clone 
        ```
        
    2. 编译
        
        ```
        cd ~/camera_ws
        catkin_make --pkg camera_bridge -j16
        ```
        
    3. 注：若编译失败，记得清理上一次的build
        
        ```
        rm -rf build/camera_bridge devel/lib/camera_bridge
        ```
        
    4. 运行
        ROS 工作空间的环境配置命令，用于让 ROS 识别当前工作空间的自定义资源
        
        ```
        source devel/setup.bash
        ```
        ROS 节点检测包单独运行
        ```
        rosrun camera_bridge k4a_detect_ros
        ```
        or 检测+串口信息传递的launch文件（USB转TTL提前插好）
        ```
        roslaunch camera_bridge k4a_and_serial.launch
        ```
        