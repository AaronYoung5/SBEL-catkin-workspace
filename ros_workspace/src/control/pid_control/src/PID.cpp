// Header file include
#include "pid_control/PID.h"

PID::PID(float kp, float ki, float kd)
    : kp_(kp), ki_(ki), kd_(kd), p_error_(0), d_error_(0), i_error_(0) {}

PID::PID() : kp_(1), ki_(0), kd_(0), p_error_(0), d_error_(0), i_error_(0) {}

void PID::updateError(float cte) {
  d_error_ = cte - p_error_;
  p_error_ = cte;
  i_error_ += cte;

  // Debug
  // std::cout << "Cross Track Error :: " << cte << std::endl;
  // std::cout << "Proportional error :: " << p_error_ << std::endl;
  // std::cout << "Integral error :: " << i_error_ << std::endl;
  // std::cout << "Derivative error :: " << d_error_ << std::endl;
}

float PID::totalError() {
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

// void PID::printVec(std::string title, Vector3D<> &vec) {
//   std::cout << title << " :: (" << vec.x() << ", " << vec.y() << ", " <<
//   vec.z()
//             << ")" << std::endl;
// }
