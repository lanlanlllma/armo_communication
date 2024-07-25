cmake_minimum_required(VERSION 3.10)

# 项目名称
project(videorecv)



# 查找OpenCV包
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# 包含目录
include_directories(${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# 指定源文件
set(SOURCES
    main.cpp
    frame_processor.cpp
    armor_detection.cpp
    armor.cpp
    number_classifier.cpp
    Quaternion.cpp
    kalman.cpp
)

# 生成可执行文件
add_executable(${PROJECT_NAME} ${SOURCES})

# 链接OpenCV库
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
