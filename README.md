# camera_bridge

## 代码部署运行步骤
1. 创建ROS工作空间` camera_ws `并初始化
    ```
    mkdir camera_ws && cd camera_ws
    catkin_init_workspace
    ```

2. 并将本仓库放在`src`下
    ```
    mkdir src && cd src
    git clone https://github.com/L-Anjing/camera
    ```
  
3. 按照`requirement.txt`内列出的依赖，逐个安装
   
4. 更改`CMakeLists.txt`依赖路径适配
    ```
    #8 9 行
    set(OpenCv_DIR "/home/li/opencv/lib/cmake/opencv4")
    set(TensorRT_ROOT "/home/li/TensorRT-8.6.1.6")
    ```
    ```
    #56行
    /home/li/TensorRT-8.6.1.6/lib
    ```

5. 编译
    ```
    cd ~/camera_ws
    catkin_make --pkg camera_bridge -j16
    ``` 
   
6. 注：若编译失败，记得清理上一次的build
    ```
    rm -rf build/camera_bridge devel/lib/camera_bridge
    ```
      
7. 运行
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
        