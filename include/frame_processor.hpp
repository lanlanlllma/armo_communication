#ifndef FRAME_PROCESSOR_HPP
#define FRAME_PROCESSOR_HPP
#include "caculation.hpp"
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include "../include/kalman.hpp"


class FrameProcessor {
public:
    FrameProcessor();

    void processFrame(cv::Mat &frame);

    std::vector<std::vector<std::vector<double>>> summary;
    Transform transform_array;
    Eigen::VectorXd x;
    Eigen::MatrixXd P, R, Q;
    Pose last_p;
    bool first_f;
    int last_frame;
    int fcount;
    int missed_frames;
    int max_missed_frames;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    void setCameraMatrix(cv::Mat cameraMatrix, cv::Mat distCoeffs);
    void setTransform(double camtranslation[] , double camrotation[],double gimtranslation[] , double gimrotation[]);

private:
    cv::Mat result_img;

};


#endif // FRAME_PROCESSOR_HPP
