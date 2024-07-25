#ifndef KALMAN_HPP
#define KALMAN_HPP

#include <Eigen/Dense>

// 状态转移函数
Eigen::VectorXd f(const Eigen::VectorXd &x);

// 状态转移函数的雅可比矩阵
Eigen::MatrixXd F_jacobian(const Eigen::VectorXd &x);

// 测量函数
Eigen::VectorXd h(const Eigen::VectorXd &x);

// 测量函数的雅可比矩阵
Eigen::MatrixXd H_jacobian(const Eigen::VectorXd &x);

// 初始化卡尔曼滤波器
void initializeKalmanFilter(Eigen::VectorXd &x, Eigen::MatrixXd &P, Eigen::MatrixXd &R, Eigen::MatrixXd &Q);

// EKF 预测步骤
void predict(Eigen::VectorXd &x, Eigen::MatrixXd &P, const Eigen::MatrixXd &Q);

// EKF 更新步骤
void update(Eigen::VectorXd &x, Eigen::MatrixXd &P, const Eigen::MatrixXd &R, const Eigen::VectorXd &z);

// 打印当前状态和协方差矩阵
void printState(const Eigen::VectorXd &x, const Eigen::MatrixXd &P);


#endif // KALMAN_HPP
