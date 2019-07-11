// ROS include
#include "ros/ros.h"

class PID {
private:
  // Private variables

  // PID Settings
  float kp_, ki_, kd_;                // PID tuning constants
  float p_error_, d_error_, i_error_; // PID errors

public:
  // Public methods

  // Constructors
  PID() : kp_(1), ki_(0), kd_(0), p_error_(0), d_error_(0), i_error_(0) {}
  PID(float kp, float ki, float kd)
      : kp_(kp), ki_(ki), kd_(kd), p_error_(0), d_error_(0), i_error_(0) {}

  // Public setters
  void setConstants(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }
  void setKp(float kp) { kp_ = kp; }
  void setKi(float ki) { ki_ = ki; }
  void setKd(float kd) { kd_ = kd; }

  // PID computation methods
  // Computes and returns the total error
  float totalError() {
    // Debug
    // std::cout << "-" << kp_ << " * " << p_error_ << " - " << kd_ << " * "
    //           << d_error_ << " - " << ki_ << " * " << i_error_ << " = "
    //           << -kp_ * p_error_ - kd_ * d_error_ - ki_ * i_error_ <<
    //           std::endl;
    // std::cout << "Total Error :: "
    //           << -kp_ * p_error_ - kd_ * d_error_ - ki_ * i_error_ <<
    //           std::endl;

    return -kp_ * p_error_ - kd_ * d_error_ - ki_ * i_error_;
  }
  // Updates the error using the cross track error (cte)
  void updateError(float cte) {
    d_error_ = cte - p_error_;
    p_error_ = cte;
    i_error_ += cte;

    // Debug
    // std::cout << "Cross Track Error :: " << cte << std::endl;
    // std::cout << "Proportional error :: " << p_error_ << std::endl;
    // std::cout << "Integral error :: " << i_error_ << std::endl;
    // std::cout << "Derivative error :: " << d_error_ << std::endl;
  }
};
