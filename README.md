# camera_bridge

# 代码部署步骤

## 相机依赖安装

### K4A相机依赖安装

务必同时安装ros驱动

https://blog.csdn.net/zxxxiazai/article/details/108152376

### Realsense相机驱动安装

https://blog.csdn.net/lalawinter/article/details/138968455

以上教程仅供参考，如有错误，自行搜索其他教程

### 其他应有的配置

包括PCL,ROS-noetic,yolov8,openCV,TensorRT,Cmake等等，小电脑应该之后会配置一下

## 代码运行步骤

1. ROS环境下的编译及运行
    1. 放到ROS的catkin_ws/src下
        
        ```
        cd ~/catkin_ws/src
        git clone 
        ```
        
    2. 编译
        
        ```
        cd ~/catkin_ws
        catkin_make --pkg camera_bridge -j16
        ```
        
    3. 注：若编译失败，记得
        
        ```
        rm -rf build/camera_bridge devel/lib/camera_bridge
        ```
        
        清理上一次的build
        
    4. 编译成功后，可执行文件会放在：
        
        ```
        ~/catkin_ws/devel/lib/camera_bridge/
        ```
        
        ```
        # 对应程序的名称
        rs_viewer_ros
        k4a_detect_ros
        
        ```
        
    5. 运行
        
        ```
        #加载ROS环境
        source /opt/ros/noetic/setup.bash
        source ~/catkin_ws/devel/setup.bash
        ```
        
        ```
        rosrun camera_bridge rs_viewer_ros
        # or
        rosrun camera_bridge rs_viewer_ros
        
        ```
        
2. 非ROS环境的编译及运行
