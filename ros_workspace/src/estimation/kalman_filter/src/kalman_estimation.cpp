#include "kalman_filter/kalman_estimation.h"

KalmanEstimation::KalmanEstimation() {
  Eigen::Matrix<float, 4, 4> F;
  F << 1, dt, 0, 0, 0, 1, 0, 0, 0, 0, 1, dt, 0, 0, 0, 1;

  Eigen::Matrix<float, 4, 4> Q;
  Q << 0, 0.001, 0, 0, 0.001, 0.001, 0, 0, 0, 0, 0, 0.001, 0, 0, 0.001, 0.001;

  Eigen::Matrix<float, 2, 4> H;
  H << 1 / .3048, 0, 0, 0, 0, 0, 1 / .3048, 0;

  Eigen::Matrix<float, 2, 2> R;
  R << 5, 0, 0, 5;

  Eigen::Matrix<float, 4, 4> P;
  P << 500, 0, 0, 0, 0, 500, 0, 0, 0, 0, 500, 0, 0, 0, 0, 500;

  // Eigen::Matrix<float, 4, 4> B;
  // B << 0;

  Eigen::Matrix<float, 4, 1> x0;
  x0 << 1000, 0, 1000, 0;

  bool debug = false;

  // filter = new KalmanFilter<float, 3, 3>(F, Q, H, R, P, dt, debug);
}

KalmanEstimation::~KalmanEstimation() { delete filter; }
