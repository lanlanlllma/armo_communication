#include "../include/caculation.hpp"

Quaternion::Quaternion() : w(0), x(0), y(0), z(0) {}

Quaternion::Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}

Transform::Transform() : yaw_gimbal(0), pitch_gimbal(0), roll_gimbal(0), roll_camera(0), yaw_camera(0), cam_tx(0) {}

Transform::Transform(double yaw_gimbal, double pitch_gimbal, double roll_gimbal, double roll_camera, double yaw_camera, double cam_tx): yaw_gimbal(yaw_gimbal), pitch_gimbal(pitch_gimbal), roll_gimbal(roll_gimbal), roll_camera(roll_camera), yaw_camera(yaw_camera), cam_tx(cam_tx) {}

Quaternion Quaternion::operator*(const Quaternion& q) const {
    return Quaternion(
        w * q.w - x * q.x - y * q.y - z * q.z,
        w * q.x + x * q.w + y * q.z - z * q.y,
        w * q.y - x * q.z + y * q.w + z * q.x,
        w * q.z + x * q.y - y * q.x + z * q.w
    );
}

Quaternion Quaternion::operator/(double n) {
    return Quaternion(w / n, x / n, y / n, z / n);
}

Quaternion Quaternion::conjugate() const {
    return Quaternion(w, -x, -y, -z) / ((w * w + x * x + y * y + z * z));
}

 Quaternion Quaternion::inverse() const {
        return Quaternion(w, -x, -y, -z);
    }

 void Quaternion::normalize() {
        double norm = std::sqrt(w*w + x*x + y*y + z*z);
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;
    }

double mod(const Quaternion& q) {
    return std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
}

Quaternion eulerToQuaternion(double yaw, double pitch, double roll) {
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    return Quaternion(
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        sy * cp * sr + cy * sp * cr,
        sy * cp * cr - cy * sp * sr
    );
}

void quaternionToEuler(const Quaternion& q, double& yaw, double& pitch, double& roll) {
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp);
    else
        pitch = std::asin(sinp);

    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

void quaternionTrans(double x, double y, double z, double& px, double& py, double& pz, const Quaternion& q) {
    px = x * (q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z) +
         y * 2 * (q.x * q.y - q.w * q.z) +
         z * 2 * (q.w * q.y + q.x * q.z);

    py = px * 2 * (q.w * q.z + q.x * q.y) +
         py * (q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z) +
         pz * 2 * (q.y * q.z - q.w * q.x);

    pz = px * 2 * (q.x * q.z - q.w * q.y) +
         py * 2 * (q.w * q.x + q.y * q.z) +
         pz * (q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
}

void transformPoint(double& x, double& y, double& z, const double& tx, const double& ty, const double& tz, const Quaternion& q, int top2bottom) {
    Quaternion p_e(0, tx, ty, tz);
    Quaternion q_conj = q.conjugate();
    Quaternion p_e_prime = q * p_e;
    p_e_prime = p_e_prime * q_conj;
    double t_x = p_e_prime.x;
    double t_y = p_e_prime.y;
    double t_z = p_e_prime.z;

    if (top2bottom == 0) {
        x += t_x;
        y += t_y;
        z += t_z;
    } else {
        x += tx;
        y += ty;
        z += tz;
    }

    Quaternion p(0, x, y, z);
    q_conj = q.conjugate();
    Quaternion p_prime = q * p;
    p_prime = p_prime * q_conj;

    x = p_prime.x - tx - t_x;
    y = p_prime.y - ty - t_y;
    z = p_prime.z - tz - t_z;
}

Pose::Pose() : x(0), y(0), z(0), q() {}

Pose::Pose(double x, double y, double z, Quaternion q) : x(x), y(y), z(z), q(q) {}

Pose calculateNewPose(double x, double y, double z, double tx, double ty, double tz, Quaternion q, double dyaw, double dpitch, double droll, int top2bottom) {
    Quaternion dq = eulerToQuaternion(dyaw, dpitch, droll);
    transformPoint(x, y, z, tx, ty, tz, dq, top2bottom);
    q = q * dq;
    return {x, y, z, q};
}

std::vector<double> cam2odom(Pose pose, Transform transform){
    Quaternion qt = pose.q;
    // std::cout<<"w: "<<qt.w<<" x: "<<qt.x<<" y: "<<qt.y<<" z: "<<qt.z<<std::endl;
    Pose pose_gimbal=calculateNewPose(pose.x,pose.y,pose.z,-transform.cam_tx,0,0,qt,transform.yaw_camera,0,transform.roll_camera,1);
    // std::cout<<"x: "<<pose_gimbal.x<<" y: "<<pose_gimbal.y<<" z: "<<pose_gimbal.z<<std::endl;
    // std::cout<<"q: "<<pose_gimbal.q.w<<" "<<pose_gimbal.q.x<<" "<<pose_gimbal.q.y<<" "<<pose_gimbal.q.z<<std::endl;
    Pose result=calculateNewPose(pose_gimbal.x,pose_gimbal.y,pose_gimbal.z,0,0,0,pose_gimbal.q,transform.yaw_gimbal,transform.pitch_gimbal,transform.roll_gimbal,1);
    double yall,pitch,roll;
    // std::cout<<"w: "<<result.q.w<<" x: "<<result.q.x<<" y: "<<result.q.y<<" z: "<<result.q.z<<std::endl;
    quaternionToEuler(result.q,yall,pitch,roll);
    // std::cout<<"yaw: "<<yall<<" pitch: "<<pitch<<" roll: "<<roll<<std::endl;
    return {result.x,result.y,result.z,yall,pitch,roll};
}

std::vector<Pose> vect2pose(const std::vector<cv::Mat>& tvec, const std::vector<cv::Mat>& rvec){
    std::vector<Pose> result;
    if(tvec.size()!=rvec.size()){
        std::cerr<<"tvec and rvec size not equal"<<std::endl;
        return {};
    };
    for(int i=0;i<tvec.size();i++){
        Pose t_pose;
        cv::Mat tvec1=tvec[i];
        cv::Mat rvec1=rvec[i];
        double x=tvec1.at<double>(0,0);
        double y=tvec1.at<double>(0,1);
        double z=tvec1.at<double>(0,2);
        Quaternion q;
        q=eulerToQuaternion(rvec1.at<double>(0,0),rvec1.at<double>(0,1),rvec1.at<double>(0,2));
        // double yaw,pitch,roll;
        // cv::Mat R;
        // cv::Rodrigues(rvec1,R);
        // cv::Mat q;
        // cv::Rodrigues(R.t(),q);
        // quaternionToEuler(Quaternion(q.at<double>(0,0),q.at<double>(0,1),q.at<double>(0,2),q.at<double>(0,3)),yaw,pitch,roll);
        t_pose.x=x;
        t_pose.y=y;
        t_pose.z=z;
        t_pose.q=q;
        // std::cout<<"x: "<<t_pose.x<<" y: "<<t_pose.y<<" z: "<<t_pose.z<<std::endl;
        result.push_back(t_pose);
    }
    // std::cout<<"size: "<<result.size()<<std::endl;
    return result;
}

Quaternion matrixToQuaternion(const cv::Mat& R) {
    double w = std::sqrt(1.0 + R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2)) / 2.0;
    double x = (R.at<double>(2,1) - R.at<double>(1,2)) / (4.0 * w);
    double y = (R.at<double>(0,2) - R.at<double>(2,0)) / (4.0 * w);
    double z = (R.at<double>(1,0) - R.at<double>(0,1)) / (4.0 * w);
    return Quaternion(w, x, y, z);
}

// 从旋转向量转换为旋转矩阵
cv::Mat rodriguesToMatrix(const cv::Mat& rvec) {
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    return R;
}

// 找到旋转中心
cv::Point3d findRotationCenter(const cv::Mat& R, const cv::Mat& t1, const cv::Mat& t2) {
    cv::Mat I = cv::Mat::eye(3, 3, CV_64F);
                        std::cout<<"找旋转轴"<<std::endl;
    cv::Mat A = I - R;
    cv::Mat b = t2 - R * t1;

    cv::Mat center;
    cv::solve(A, b, center, cv::DECOMP_SVD);
    
    return cv::Point3d(center.at<double>(0), center.at<double>(1), center.at<double>(2));
}
// 线速度
std::vector<double> calliner_speed(std::vector<double> poseuler, std::vector<double> last_poseeuler,int time){
    std::vector<double> speed;
    for(int i=0;i<3;i++){
        speed.push_back((poseuler[i]-last_poseeuler[i])/time);
    }
    return speed;
}
// 角速度
std::vector<double> caangle_speed(std::vector<double> poseuler, std::vector<double> last_poseeuler, int time){
    std::vector<double> speed;
    for(int i=3;i<poseuler.size();i++){
        speed.push_back((poseuler[i]-last_poseeuler[i])/time);
    }
    return speed;
}
// 速度的模
double speed(std::vector<double> speed){
    return sqrt(speed[0]*speed[0]+speed[1]*speed[1]+speed[2]*speed[2]);
}

