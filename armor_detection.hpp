#ifndef ARMOR_DETECTION_HPP
#define ARMOR_DETECTION_HPP

#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;

const float SMALL_ARMOR_WIDTH = 0.135;
const float SMALL_ARMOR_HEIGHT = 0.055;
const std::vector<Point3f> SMALL_ARMOR_POINTS = {
    {0, +SMALL_ARMOR_WIDTH / 2, -SMALL_ARMOR_HEIGHT / 2},
    {0, +SMALL_ARMOR_WIDTH / 2, +SMALL_ARMOR_HEIGHT / 2},
    {0, -SMALL_ARMOR_WIDTH / 2, +SMALL_ARMOR_HEIGHT / 2},
    {0, -SMALL_ARMOR_WIDTH / 2, -SMALL_ARMOR_HEIGHT / 2}
};

const Mat cameraMatrix = (Mat_<double>(3, 3) << 
                    2102.080562187802, 0, 689.2057889332623,
                    0, 2094.0179120166754, 496.6622802275393,
                    0, 0, 1);
const Mat distCoeffs = (Mat_<double>(1, 5) << 
                    -0.06478109387525666,
                    0.39036067923005396,
                    -0.0042514793151166306,
                    0.008306749648029776,
                    -1.6613800909405605);
const float target_ratio = (3.0f);
const float tolerance = 3.f;
const float angle_tolerance = 3.0f;
const float l_target_raito = (1.7f);
const float l_tolerance = 1.f;
const std::string model_path = "../0709work/model/mlp.onnx";
const std::string label_path = "../0709work/model/label.txt";
const float confidence_threshold = 0.9f;
const std::vector<std::string> ignore_classes={"outpost","guard","base"};
void adjustBrightnessContrast(const Mat& src, Mat& dst, double alpha, int beta);
void processImage(const Mat& img, Mat& drawing, std::vector<Point2f>& armorPoints);
void processArmorDetection(const cv::Mat& img, std::vector<cv::Mat> &tvec, std::vector<cv::Mat> &rvec, std::vector<std::string> &result);

#endif // ARMOR_DETECTION_HPP
