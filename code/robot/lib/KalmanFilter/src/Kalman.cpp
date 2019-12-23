#include "Kalman.hpp"

KalmanFilter::KalmanFilter(
        float dt,
        const Eigen::MatrixXf& A,
        const Eigen::MatrixXf& B,
        const Eigen::MatrixXf& C,
        const Eigen::MatrixXf& Q,
        const Eigen::MatrixXf& R,
        const Eigen::MatrixXf& P)
    : A(A), B(B), C(C), Q(Q), R(R), P0(P),
    m(C.rows()), n(A.rows()), dt(dt), initialized(false),
    I(n, n), x_hat(n), x_hat_new(n) {
    I.setIdentity();
}


void KalmanFilter::init(float t0, const Eigen::VectorXf& x0) {
    x_hat = x0;
    P = P0;
    this->t0 = t0;
    t = t0;
    initialized = true;
}

void KalmanFilter::init() {
    x_hat.setZero();
    P = P0;
    t0 = 0;
    t = t0;
    initialized = true;
}

bool KalmanFilter::update(const Eigen::VectorXf& y, float u) {
    if(!initialized) return false;
    // Prediction
    x_hat_new = A * x_hat + B * u;
    P = A * P * A.transpose() + Q;
    // Correction
    K = P * C.transpose() * (C * P * C.transpose() + R).inverse();
    x_hat_new += K * (y - C * x_hat_new);
    P = (I - K * C) * P;
    // Step forwared in time
    x_hat = x_hat_new;
    t += dt;
    return true;
}


bool KalmanFilter::update(const Eigen::VectorXf& y, float u, float dt, const Eigen::MatrixXf A) {
    this->A = A;
    this->dt = dt;
    return this->update(y, u);
}