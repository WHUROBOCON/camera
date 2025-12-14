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
        
        ```
        #加载ROS环境
        source devel/setup.bash
        ```
        
        ```
        rosrun camera_bridge k4a_detect_ros
        # or
        roslaunch camera_bridge k4a_and_serial.launch
        
        ```
        