#ifndef KALMAN_HPP_
#define KALMAN_HPP_

#include <Eigen.h>
#include <Eigen/Dense>

class KalmanFilter {
public:
    KalmanFilter(
        float dt,
        const Eigen::MatrixXf& A,
        const Eigen::MatrixXf& B,
        const Eigen::MatrixXf& C,
        const Eigen::MatrixXf& Q,
        const Eigen::MatrixXf& R,
        const Eigen::MatrixXf& P
    );
    KalmanFilter();
    void init();
    void init(float t0, const Eigen::VectorXf& x0);
    bool update(const Eigen::VectorXf& y, float u);
    bool update(const Eigen::VectorXf& y, float u, float dt, const Eigen::MatrixXf A);
    Eigen::VectorXf state() {
        return x_hat;
    };
    float time() {
        return t;
    };
private:
    Eigen::MatrixXf A, B, C, Q, R, P, K, P0;
    int16_t m, n;
    float t0, t;
    float dt;
    bool initialized;
    Eigen::MatrixXf I;
    Eigen::VectorXf x_hat, x_hat_new;

};

#endif