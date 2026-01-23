cmake_minimum_required(VERSION 3.10)
project(camera_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- 基础依赖 ----
set(OpenCv_DIR "/home/li/opencv/lib/cmake/opencv4")
set(TensorRT_ROOT "/home/li/TensorRT-8.6.1.6")

find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(PCL REQUIRED)
find_package(k4a REQUIRED)
find_package(CUDA REQUIRED)
find_package(yaml-cpp REQUIRED)

# ---- 检查ROS环境 ----
if(DEFINED ENV{ROS_DISTRO} OR DEFINED CATKIN_DEVEL_PREFIX)
    message(STATUS "🔹 Detected ROS, building with ROS support")
    set(BUILD_WITH_ROS ON)

    find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        cv_bridge
        image_transport
        serial
    )

    catkin_package(
        CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge image_transport
    )
else()
    message(STATUS "🔸 No ROS detected, building without ROS")
    set(BUILD_WITH_ROS OFF)
endif()

include_directories(
    ${catkin_INCLUDE_DIRS}
    /usr/local/cuda/include
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${TensorRT_ROOT}/include
    ${PROJECT_SOURCE_DIR}/inc
)

if(BUILD_WITH_ROS)
    include_directories(${catkin_INCLUDE_DIRS})
endif()


# ---- 链接路径 ----
link_directories(
    /home/li/TensorRT-8.6.1.6/lib
    /usr/local/cuda/lib64
    /usr/lib/x86_64-linux-gnu
)

# ---- YOLO子模块 ----
add_subdirectory(src/yolo)


#通用可执行文件
set(COMMON_LIBS
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    nvinfer
    nvonnxparser
    ${CUDA_LIBRARIES}
    yolo
    -lffi
    yaml-cpp
)
#---realsense---
add_executable(rs_viewer src/common/camera_params.cpp src/realsense/rs_detect.cpp src/realsense/realsense.cpp
    src/utils/vision_draw.cpp
    src/utils/myinfer.cpp src/yolo/yolo.cu)
target_link_libraries(rs_viewer PRIVATE

    realsense2::realsense2
    ${COMMON_LIBS}
)


#---K4A---
add_executable(k4a_detect src/common/camera_params.cpp src/k4a/k4a_detect.cpp src/k4a/camera_k4a.cpp
    src/utils/myinfer.cpp src/utils/vision_draw.cpp src/utils/block_recognizer.cpp src/yolo/yolo.cu)
target_link_libraries(k4a_detect PRIVATE

    k4a::k4a
    ${COMMON_LIBS}
)



#--k4a_record_videos--
add_executable(record_videos src/common/camera_params.cpp src/k4a/k4a_capture_videos.cpp  src/k4a/camera_k4a.cpp src/utils/vision_draw.cpp)
target_link_libraries(record_videos PRIVATE

    k4a::k4a
    ${COMMON_LIBS}
)
#--k4a_capture_pictures--
add_executable(capture_images src/common/camera_params.cpp src/k4a/k4a_capture_images.cpp src/k4a/camera_k4a.cpp src/utils/vision_draw.cpp)
target_link_libraries(capture_images PRIVATE

    k4a::k4a
    ${COMMON_LIBS}
)



#             ROS可执行文件（仅在ROS下编译）
if(BUILD_WITH_ROS)
    #---k4a_detect_ros---
    add_executable(k4a_detect_ros src/common/camera_params.cpp src/k4a/k4a_detect.cpp src/k4a/camera_k4a.cpp src/utils/myinfer.cpp
        src/utils/vision_draw.cpp src/utils/block_recognizer.cpp src/yolo/yolo.cu)
    target_compile_definitions(k4a_detect_ros PRIVATE BUILD_WITH_ROS)
    target_include_directories(k4a_detect_ros PRIVATE
        ${catkin_INCLUDE_DIRS}/inc
    )
    target_link_libraries(k4a_detect_ros PRIVATE
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        nvinfer
        nvonnxparser
        k4a::k4a
        ${CUDA_LIBRARIES}
        yolo
        pthread
        -lffi
        yaml-cpp
    )

    install(TARGETS k4a_detect_ros
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

    #---rs_viewer_ros---
    add_executable(rs_viewer_ros src/common/camera_params.cpp src/realsense/rs_detect.cpp src/realsense/realsense.cpp
        src/utils/myinfer.cpp src/yolo/yolo.cu
        src/utils/vision_draw.cpp
    )
    target_compile_definitions(rs_viewer_ros PRIVATE BUILD_WITH_ROS)
    target_include_directories(rs_viewer_ros PRIVATE ${catkin_INCLUDE_DIRS}/inc)
    target_link_libraries(rs_viewer_ros PRIVATE
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        nvinfer
        nvonnxparser
        realsense2::realsense2
        ${CUDA_LIBRARIES}
        yolo
        pthread
        -lffi
        yaml-cpp
    )
    install(TARGETS rs_viewer_ros RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



endif()
