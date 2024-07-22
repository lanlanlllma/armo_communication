#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>

struct Quaternion {
    double w, x, y, z;

    Quaternion();
    Quaternion(double w, double x, double y, double z);

    Quaternion operator*(const Quaternion& q) const;
    Quaternion operator/(double n);
    Quaternion conjugate() const;
    Quaternion inverse() const;
    void normalize();

    friend double mod(const Quaternion& q);
};

struct Transform{
    double yaw_gimbal, pitch_gimbal, roll_gimbal;
    double roll_camera, yaw_camera;

    double cam_tx;

    Transform();
    Transform(double yaw_gimbal, double pitch_gimbal, double roll_gimbal, double roll_camera, double yaw_camera, double cam_tx);
};



Quaternion eulerToQuaternion(double yaw, double pitch, double roll);
void quaternionToEuler(const Quaternion& q, double& yaw, double& pitch, double& roll);

void quaternionTrans(double x, double y, double z, double& px, double& py, double& pz, const Quaternion& q);

void transformPoint(double& x, double& y, double& z, const double& tx, const double& ty, const double& tz, const Quaternion& q, int top2bottom = 1);

struct Pose {
    double x, y, z;
    Quaternion q;

    Pose();
    Pose(double x, double y, double z, Quaternion q);
};
std::vector<double> cam2odom(Pose pose, Transform transform);

Pose calculateNewPose(double x, double y, double z, double tx, double ty, double tz, Quaternion q, double dyaw, double dpitch, double droll, int top2bottom);

std::vector<Pose> vect2pose(const std::vector<cv::Mat>& tvec, const std::vector<cv::Mat>& rvec);

cv::Mat rodriguesToMatrix(const cv::Mat& rvec);
Quaternion matrixToQuaternion(const cv::Mat& R);
cv::Point3d findRotationCenter(const cv::Mat& R, const cv::Mat& t1, const cv::Mat& t2);
std::vector<double> calliner_speed(std::vector<double> poseuler, std::vector<double> last_poseeuler,int time);
std::vector<double> caangle_speed(std::vector<double> poseuler, std::vector<double> last_poseeuler, int time);
double speed(std::vector<double> speed);
#endif // QUATERNION_HPP
