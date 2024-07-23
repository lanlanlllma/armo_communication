#include "frame_processor.hpp"
#include "armor_detection.hpp"
#include <iostream>

// Other necessary includes

FrameProcessor::FrameProcessor() 
    : transform_array(0, 0, 0.0, 1.57, 1.57, 0.2),
      first_f(true),
      last_frame(0),
      fcount(0),
      missed_frames(0),
      max_missed_frames(3), // Set the number of frames to allow predictions
      cameraMatrix(cv::Mat::zeros(3,3,CV_16F)),
      distCoeffs(cv::Mat::zeros(1,5,CV_16F))

{
    result_img = cv::Mat::zeros(720, 1280, CV_8UC3);
    summary.resize(2); // Initialize the summary to hold two 2D vectors
    initializeKalmanFilter(x, P, R, Q);
}

void FrameProcessor::processFrame(cv::Mat &frame) {
    cv::circle(result_img, cv::Point(640, 360), 5, cv::Scalar(0, 255, 0), -1);
    std::vector<cv::Mat> tvec, rvec;
    std::vector<std::string> result;
    std::vector<double> poseuler;

    processArmorDetection(frame, tvec, rvec, result,cameraMatrix, distCoeffs);

    if (!tvec.empty()) {
        missed_frames = 0;
        std::vector<Pose> pose = vect2pose(tvec, rvec);
        int count = 0;
        if (first_f) {
            last_p = pose[0];
            first_f = false;
        }
        bool flag = false;
        double distance, max_distance = 0;
        int max_index = 0;

        for (auto &p : pose) {
            if (result[count][0] == 'n') {
                count++;
                continue;
            }
            distance = pow(p.x - last_p.x, 2) + pow(p.y - last_p.y, 2) + pow(p.z - last_p.z, 2);
            if (!flag) {
                flag = true;
                max_distance = distance;
                max_index = 0;
                count++;
                continue;
            }
            if (distance > max_distance) {
                max_distance = distance;
                max_index = count;
                count++;
            }
        }

        if (result[max_index][0] != 'n') {
            Pose p = pose[max_index];
            std::vector<double> center_pos(3, 0.0);
            std::vector<Eigen::VectorXd> measurement;
            std::vector<double> liner_speed;
            std::vector<double> angle_speed;
            double rotation_r;
            poseuler = cam2odom(p, transform_array);
            summary[0].push_back(poseuler);
            std::cout << fcount << " result: " << result[max_index] << " x: " << poseuler[0] << " y: " << poseuler[1] << " z: " << poseuler[2] << " yaw: " << poseuler[3] << " pitch: " << poseuler[4] << " roll: " << poseuler[5] << std::endl;
            std::vector<double> last_poseuler = {0, 0, 0, 0, 0, 0};
            last_poseuler[0] = last_p.x;
            last_poseuler[1] = last_p.y;
            last_poseuler[2] = last_p.z;
            quaternionToEuler(last_p.q, last_poseuler[3], last_poseuler[4], last_poseuler[5]);
            cv::Mat current_rvec = (cv::Mat_<double>(3, 1) << poseuler[3], poseuler[4], poseuler[5]);
            cv::Mat last_rvec = (cv::Mat_<double>(3, 1) << last_poseuler[3], last_poseuler[4], last_poseuler[5]);
            liner_speed = calliner_speed(poseuler, last_poseuler, fcount - last_frame);
            angle_speed = caangle_speed(poseuler, last_poseuler, fcount - last_frame);
            rotation_r = speed(liner_speed) / speed(angle_speed);

            std::cout << "rotation r: " << rotation_r << std::endl;
            center_pos[0] = poseuler[0] + liner_speed[0] / speed(liner_speed) * rotation_r;
            center_pos[1] = poseuler[1] + liner_speed[1] / speed(liner_speed) * rotation_r;
            center_pos[2] = poseuler[2];

            measurement.push_back(Eigen::VectorXd(3));
            measurement[0] << center_pos[0], center_pos[1], center_pos[2];
            update(x, P, R, measurement[0]);
            summary[1].push_back(center_pos);
            last_p = p;
            last_frame = fcount;

            cv::circle(result_img, cv::Point(640 + poseuler[1] * 100, 360 - poseuler[0] * 100), 5, cv::Scalar(0, 0, 255), -1);
            cv::arrowedLine(result_img, cv::Point(640 + poseuler[1] * 100, 360 - poseuler[0] * 100), cv::Point(640 + center_pos[1] * 100, 360 - center_pos[0] * 100), cv::Scalar(0, 255, 0), 2);
            cv::imshow("result_img", result_img);
        } else {
            summary[0].push_back(std::vector<double>(6, 0.0));
            summary[1].push_back(std::vector<double>(3, 0.0));
        }
    } else {
        missed_frames++;
        if (missed_frames <= max_missed_frames) {
            std::vector<double> predicted_pose(6, 0.0);
            for (int i = 0; i < 3; ++i) {
                predicted_pose[i] = x[i];
            }
            summary[0].push_back(predicted_pose);
            summary[1].push_back(std::vector<double>(3, 0.0));
        } else {
            summary[0].push_back(std::vector<double>(6, 0.0));
            summary[1].push_back(std::vector<double>(3, 0.0));
        }
    }
    fcount++;
    predict(x, P, Q);
    cv::circle(result_img, cv::Point(640 + x[1] * 100, 360 - x[0] * 100), 5, cv::Scalar(255, 255, 255), -1);
    cv::imshow("result_img", result_img);
}

void FrameProcessor::setCameraMatrix(cv::Mat cameraMatrix, cv::Mat distCoeffs) {
    this->cameraMatrix = cameraMatrix;
    this->distCoeffs = distCoeffs;
}

void FrameProcessor::setTransform(double camtranslation[] , double camrotation[],double gimtranslation[] , double gimrotation[]){
    transform_array = Transform(gimtranslation[0], gimtranslation[1], gimtranslation[2], camrotation[2],camrotation[0],camtranslation[0]);
}
