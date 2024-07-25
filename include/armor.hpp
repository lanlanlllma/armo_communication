#ifndef ARMOR_HPP
#define ARMOR_HPP

#include <opencv2/opencv.hpp>
#include <string>

namespace armor {

enum class ArmorType {
    SMALL,
    LARGE
};

struct Light {
    cv::Point2f top;
    cv::Point2f bottom;
};

struct Armor {
    Light left_light;
    Light right_light;
    ArmorType type;
    cv::Mat number_image;  // 存储提取的数字图像
    std::string number;    // 识别出的数字
    double classification_confidence;  // 识别置信度
    std::string classification_result; // 识别结果字符串
};

} // namespace armor

#endif // ARMOR_HPP
