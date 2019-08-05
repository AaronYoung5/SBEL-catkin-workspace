/**
 * Kalman filter implementation using Eigen. Based on the following
 * introductory book:
 *
 *     https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python
 *
 * @author: Aaron Young
 * @date: July 22nd, 2019
 */

#include <iostream>
#include <Eigen/Dense>

#pragma once

template <class Scalar, int n, int m> class KalmanFilter {

public:
  /**
   * Create a Kalman filter with the specified matrices.
   *   F - Transition Function
   *   Q - Process noise covariance
   *   H - Measurement Function
   *   R - Measurement noise covariance
   *   P - Estimate error covariance
   */
  KalmanFilter(const Eigen::Matrix<Scalar, n, n> &F,
               const Eigen::Matrix<Scalar, n, n> &Q,
               const Eigen::Matrix<Scalar, m, n> &H,
               const Eigen::Matrix<Scalar, m, m> &R,
               const Eigen::Matrix<Scalar, n, n> &P, Scalar dt, bool debug)
      : F(F), Q(Q), H(H), R(R), P0(P), dt(dt), debug(debug),
        initialized(false) {
    if (debug) {
      std::cout << "Creating Kalman Filter Object" << std::endl;
      std::cout << "F :: " << std::endl << F << std::endl;
      std::cout << "Q :: " << std::endl << Q << std::endl;
      std::cout << "H :: " << std::endl << H << std::endl;
      std::cout << "R :: " << std::endl << R << std::endl;
      std::cout << "P :: " << std::endl << P << std::endl;
    }
    I.setIdentity();
  }

  /**
   * Create a blank estimator.
   */
  KalmanFilter() {}

  /**
   * Initialize the filter with initial states as zero.
   */
  void init() {
    if (debug)
      std::cout << "Initializing Kalman Filter" << std::endl;
    x_hat.setZero();
    P = P0;
    t0 = 0;
    t = t0;
    initialized = true;
  }

  /**
   * Initialize the filter with a guess for initial states.
   */
  void init(const Eigen::Matrix<Scalar, n, 1> &x0, double t0) {
    x_hat = x0;
    P = P0;
    this->t0 = t0;
    t = t0;
    initialized = true;
  }

  /**
   * 1. Use Precess model to predict state at the next time step
   * 2. Adjust belief to account for the uncertaint in prediction
   */
  void predict() {
    x_hat = F * x_hat;
    P = F * P * F.transpose() + Q;
  }

  /**
   * 1. Get a measurement and associated belief about its accuracy
   * 2. Compute residual between estimated state and measurement
   * 3. Compute scaling factor based on whether the measurement or prediction is
   * more accurate
   * 4. set state between the prediction and measurement based on scaling factor
   * 5. update belief in the state based on how certain we are in the
   * measurement
   */
  void update(const Eigen::Matrix<Scalar, m, 1> &z) {

    if (!initialized)
      throw std::runtime_error("Filter is not initialized!");

    y = z - H * x_hat;
    K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    x_hat += K * y;
    P = (I - K * H) * P;

    t += dt;
  }

  /**
   * Update the estimated state based on measured values,
   * using the given time step and state transition function.
   */
  void update(const Eigen::Matrix<Scalar, m, 1> &z, double dt,
              const Eigen::Matrix<Scalar, n, n> F) {
    this->F = F;
    this->dt = dt;
    update(z);
  }

  /**
   * Return the current state and time.
   */
  Eigen::Matrix<Scalar, n, 1> state() { return x_hat; };
  Scalar time() { return t; };

private:
  // Matrices for computation
  Eigen::Matrix<Scalar, n, n> F;
  Eigen::Matrix<Scalar, n, n> Q;
  Eigen::Matrix<Scalar, m, n> H;
  Eigen::Matrix<Scalar, m, m> R;
  Eigen::Matrix<Scalar, n, n> P;
  Eigen::Matrix<Scalar, n, n> P0;
  Eigen::Matrix<Scalar, n, m> K;

  // Residual
  Eigen::Matrix<Scalar, m, 1> y;

  // n-size identity
  Eigen::Matrix<Scalar, n, n> I;

  // Estimated state
  Eigen::Matrix<Scalar, n, 1> x_hat;

  // Initial and current time
  Scalar t0, t;

  // Discrete time step
  Scalar dt;

  // Print debug information?
  bool debug;

  // Is the filter initialized?
  bool initialized;
};
