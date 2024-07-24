#include <iostream>
#include <opencv2/opencv.hpp>
#include "armor_detection.hpp"
#include "number_classifier.hpp"
#include "armor.hpp"
#include <omp.h>
using namespace cv;
using namespace armor;

// void adjustBrightnessContrast(const Mat& src, Mat& dst, double alpha, int beta){
//     Mat new_image = Mat::zeros(src.size(), src.type());
//     for (int y = 0; y < src.rows; y++) {
//         for (int x = 0; x < src.cols; x++) {
//             for (int c = 0; c < 3; c++) {
//                 new_image.at<Vec3b>(y, x)[c] =
//                     saturate_cast<uchar>(alpha * src.at<Vec3b>(y, x)[c] + beta);
//             }
//         }
//     }
//     dst = new_image;
// }

// #include <opencv2/core/parallel/parallel_for.hpp>

class BrightnessContrastAdjuster : public ParallelLoopBody
{
public:
    BrightnessContrastAdjuster(const Mat &src, Mat &dst, double alpha, int beta)
        : src_(src), dst_(dst), alpha_(alpha), beta_(beta) {}

    virtual void operator()(const Range &range) const override
    {
        for (int y = range.start; y < range.end; y++)
        {
            for (int x = 0; x < src_.cols; x++)
            {
                for (int c = 0; c < 3; c++)
                {
                    dst_.at<Vec3b>(y, x)[c] = saturate_cast<uchar>(alpha_ * src_.at<Vec3b>(y, x)[c] + beta_);
                }
            }
        }
    }

private:
    const Mat &src_;
    Mat &dst_;
    double alpha_;
    int beta_;
};

cv::Mat rotationMatrixToEulerAnglesMat(const cv::Mat &R) {
    float sy = std::sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));
    
    bool singular = sy < 1e-6; // 如果 sy 接近于零，则矩阵接近奇异
    
    float x, y, z;
    if (!singular) {
        x = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        y = std::atan2(-R.at<double>(2, 0), sy);
        z = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    } else {
        x = std::atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = std::atan2(-R.at<double>(2, 0), sy);
        z = 0;
    }

    cv::Mat eulerAngles = (cv::Mat_<double>(3, 1) << x, y, z);
    return eulerAngles;
}

void adjustBrightnessContrast(const Mat &src, Mat &dst, double alpha, int beta)
{
    dst = Mat::zeros(src.size(), src.type());
    BrightnessContrastAdjuster body(src, dst, alpha, beta);
    parallel_for_(Range(0, src.rows), body);
}

void processImage(const Mat &img, Mat &drawing, std::vector<std::vector<Point2f>> &armorPoints)
{
    Mat gray, binary;
    std::vector<Mat> channels;
    split(img, channels);
    gray = channels[0];
    threshold(gray, binary, 180, 255, THRESH_BINARY | THRESH_OTSU);

    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    // Mat element2 = getStructuringElement(MORPH_RECT, Size(2, 2));
    // Mat element5 = getStructuringElement(MORPH_RECT, Size(5, 5));
    erode(binary, binary, element, Point(-1, -1), 2);
    // erode(binary, binary, element, Point(-1, -1), 2);
    // dilate(binary, binary, element2, Point(-1, -1), 2);
    // dilate(binary, binary, element2, Point(-1, -1), 2);
    dilate(binary, binary, element, Point(-1, -1), 2);

    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    findContours(binary, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    drawing = Mat::zeros(binary.size(), CV_8UC3);
    // drawing=img;
    for (int i = 0; i < contours.size(); i++) {
        Scalar color = Scalar(0, 0, 255);
        drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
    }

    std::vector<std::pair<RotatedRect, std::vector<Point2f>>> rects;
    for (const auto &contour : contours)
    {
        // noise reduce
        if (contour.size() < 5)
            continue;

        RotatedRect rotatedRect = minAreaRect(contour);
        Point2f vertices[4];
        rotatedRect.points(vertices);

        Point2f midPoint1, midPoint2;
        if (norm(vertices[0] - vertices[1]) < norm(vertices[1] - vertices[2]))
        {
            midPoint1 = (vertices[0] + vertices[1]) * 0.5f;
            midPoint2 = (vertices[2] + vertices[3]) * 0.5f;
        }
        else
        {
            midPoint1 = (vertices[1] + vertices[2]) * 0.5f;
            midPoint2 = (vertices[3] + vertices[0]) * 0.5f;
        }

        // 计算长宽比
        float width = rotatedRect.size.width;
        float height = rotatedRect.size.height;
        if (width == 0 || height == 0)
            continue;
        float aspect_ratio = max(width, height) / min(width, height);
        // std::cout << "aspect_ratio: " << aspect_ratio << std::endl;

        // 检查长宽比是否在目标范围内
        if (abs(aspect_ratio - target_ratio) < tolerance)
        {
            rects.emplace_back(rotatedRect, std::vector<Point2f>{midPoint1, midPoint2});
        }
    }
    std::sort(rects.begin(), rects.end(), [](const auto &a, const auto &b)
              { return a.first.center.x < b.first.center.x; });

    for (size_t i = 0; i < rects.size(); ++i)
    {
        for (size_t j = i + 1; j < rects.size(); ++j)
        {
            RotatedRect rect1 = rects[i].first;
            RotatedRect rect2 = rects[j].first;

            float angle1 = rect1.angle;
            float angle2 = rect2.angle;
            // std::cout<<"angle1: "<<angle1<<std::endl;
            // std::cout<<"angle2: "<<angle2<<std::endl;
            // std::cout<<std::endl;

            if (abs(angle1 - angle2) < angle_tolerance || abs((angle1 + angle2) - 90) < angle_tolerance)
            { // 判断旋转矩形是否近似平行    ||abs(abs(angle1 - angle2)-90) < angle_tolerance
                std::vector<Point2f> points;
                // left bottom first clockwise
                // find left first
                if (rects[i].second[0].x < rects[j].second[0].x)
                {
                    points.push_back(rects[i].second[1]);
                    points.push_back(rects[i].second[0]);
                }
                else
                {
                    points.push_back(rects[i].second[0]);
                    points.push_back(rects[i].second[1]);
                }
                // find right
                if (rects[i].second[1].x > rects[j].second[1].x)
                {
                    points.push_back(rects[j].second[1]);
                    points.push_back(rects[j].second[0]);
                }
                else
                {
                    points.push_back(rects[j].second[0]);
                    points.push_back(rects[j].second[1]);
                }
                // 判断大矩形的长宽比
                float l_length = abs(max(cv::norm(rects[i].second[1] - rects[j].second[1]), cv::norm(rects[i].second[0] - rects[j].second[0])));
                float l_width = abs(min(cv::norm((rects[i].second[1] - rects[i].second[0])), cv::norm((rects[j].second[1] - rects[j].second[0]))));
                // std::cout<<"length: "<<l_length<<std::endl;
                // std::cout<<"width: "<<l_width<<std::endl;
                if (l_width == 0 || l_length == 0)
                    continue;
                float l_aspect_ratio = max(l_width, l_length) / min(l_width, l_length);
                // std::cout << "aspect_ratio: " << l_aspect_ratio << std::endl;
                if (abs(l_aspect_ratio - l_target_raito) > l_tolerance)
                {
                    continue;
                }

                // points.push_back(rects[i].second[0]);
                // points.push_back(rects[i].second[1]);
                // points.push_back(rects[j].second[0]);
                // points.push_back(rects[j].second[1]);
                armorPoints.push_back(points);

                // 绘制组合的矩形
                Point2f vertices1[4], vertices2[4];
                rect1.points(vertices1);
                rect2.points(vertices2);
                // for (int k = 0; k < 4; k++) {
                //     line(drawing, vertices1[k], vertices1[(k + 1) % 4], Scalar(0, 255, 0), 2);
                //     line(drawing, vertices2[k], vertices2[(k + 1) % 4], Scalar(0, 255, 0), 2);
                // }
                for (const auto &point : points)
                {
                    circle(drawing, point, 5, Scalar(255, 0, 0), -1);
                }
            }
        }
    }
}

void processArmorDetection(const cv::Mat &img, std::vector<cv::Mat> &tvec, std::vector<cv::Mat> &rvec, std::vector<std::string> &result,cv::Mat cameraMatrix,cv::Mat distCoeffs)
{
    armor::NumberClassifier number_classifier(model_path, label_path, confidence_threshold, ignore_classes);
    std::vector<armor::Armor> armors;
    struct armor::Armor temp_armor;

    // 读取图像
    cv::Mat clsfy_img;
    // adjustBrightnessContrast(img, clsfy_img, 1.5, 15);
    clsfy_img = img;
    // cv::imshow("clsfy_img", clsfy_img);
    // std::cout << "clsfy_img.size: " <<std::endl;
    // 调整图像亮度和对比度
    cv::Mat new_image;
    // adjustBrightnessContrast(img, new_image, 0.2, -30);
    new_image = img;
    // img = new_image;
    // cv::imshow("img", img);

    // 处理图像
    cv::Mat drawing;
    std::vector<std::vector<Point2f>> matchedArmorPoints;
    processImage(new_image, drawing, matchedArmorPoints);
    // std::cout<<"找点"<<std::endl;

    if (matchedArmorPoints.size() == 0 || matchedArmorPoints[0].size() < 4)
    {
        // std::cerr << "未找到足够的点进行solvePnP!" << std::endl;
        // return;
    }
    // else{
    //     std::cout<<"找到足够的点"<<std::endl;
    // }
    for (const auto &armorPoints : matchedArmorPoints)
    {
        if (armorPoints.size() != 4)
            continue;

        line(drawing, armorPoints[0], armorPoints[2], Scalar(0, 0, 255), 2);
        line(drawing, armorPoints[1], armorPoints[3], Scalar(0, 0, 255), 2);
        for (const auto &point : armorPoints)
        {
            circle(drawing, point, 5, Scalar(0, 255, 0), -1);
            cv::putText(drawing, std::to_string(&point - &armorPoints[0]), point, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
        }
        // addWeighted(clsfy_img, 0.7, drawing, 0.3, 0, drawing);
        cv::Mat t_tvec;
        cv::Mat t_rvec;

        solvePnP(SMALL_ARMOR_POINTS, armorPoints, cameraMatrix, distCoeffs, t_rvec, t_tvec);
        cv::Mat rotationMatrix;
        cv::Rodrigues(t_rvec, rotationMatrix);
        cv::Mat eulerAngles = rotationMatrixToEulerAnglesMat(rotationMatrix);
        // std::cout << "rvec: " << t_rvec << std::endl;
        // std::cout << "tvec: " << t_tvec << std::endl;
        tvec.push_back(t_tvec);
        rvec.push_back(eulerAngles);

        Armor temp_armor;
        temp_armor.left_light.top = armorPoints[1];
        temp_armor.left_light.bottom = armorPoints[0];
        temp_armor.right_light.top = armorPoints[2];
        temp_armor.right_light.bottom = armorPoints[3];
        temp_armor.type = armor::ArmorType::SMALL;
        armors.push_back(temp_armor);

        // Assuming number_classifier is defined and initialized somewhere
        number_classifier.ExtractNumbers(clsfy_img, armors);
        number_classifier.Classify(armors);
        // std::cout << armors[0].classification_result << std::endl;
        // if(armors[0].classification_result[0]!='n')
        result.push_back(armors[0].classification_result);

        // cv::putText(drawing, armors[0].classification_result, armorPoints[0], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
    }
    addWeighted(clsfy_img, 0.7, drawing, 0.3, 0, drawing);
    // line(drawing, Point(0, 540), Point(1920, 540), Scalar(0, 255, 0), 2);
    // line(drawing, Point(960, 0), Point(960, 1080), Scalar(0, 255, 0), 2);

    cv::imshow("drawing", drawing);
}