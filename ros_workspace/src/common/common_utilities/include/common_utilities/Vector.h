#pragma once

#include <math.h>

namespace common_utilities {
template <class T = float> struct Vector2D {
  T x;
  T y;
};

template <class T = float> class Vector3D {
private:
  T x_;
  T y_;
  T z_;

public:
  Vector3D(T x, T y, T z) : x_(x), y_(y), z_(z) {}

  T x() { return x_; }
  T y() { return y_; }
  T z() { return z_; }

  void setX(T x) { x_ = x; }
  void setY(T y) { y_ = y; }
  void setZ(T z) { z_ = z; }

  Vector3D<T> getUnit() {
    return Vector3D<T>(x_ / length(), y_ / length(), z_ / length());
  }

  T dot(Vector3D<T> vec) { return x_ * vec.x() + y_ * vec.y(); }

  Vector3D<T> cross(Vector3D<T> vec) {
    return Vector3D<T>(y_ * vec.z() - z_ * vec.y(), vec.x() * z_ - x_ * vec.z(),
                       x_ * vec.y() - y_ * vec.x());
  }

  T length() { return sqrt(pow(x_, 2) + pow(y_, 2)); }
  T lengthSquared() { return pow(x_, 2) + pow(y_, 2); }

  Vector3D<T> operator*(T n) { return Vector3D<T>(x_ * n, y_ * n, z_ * n); }

  Vector3D<T> operator-(Vector3D<T> vec) {
    return Vector3D<T>(x_ - vec.x(), y_ - vec.y(), z_ - vec.z());
  }

  Vector3D<T> operator+(Vector3D<T> vec) {
    return Vector3D<T>(x_ + vec.x(), y_ + vec.y(), z_ + vec.z());
  }
};
} // namespace common_utilities
