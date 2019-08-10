#pragma once

#include "Vector.h"
#include <math.h>

namespace common_utilities {
template <class T = float> class Triangle {
private:
  Vec2<T> A_;
  Vec2<T> B_;
  Vec2<T> C_;

  T a_;
  T b_;
  T c_;

public:
  Triangle(Vec2<T> A = 0, Vec2<T> B = 0, Vec2<T> C = 0) : A_(A), B_(B), C_(C) {
    a_ = (A - B).length();
    b_ = (B - C).length();
    c_ = (C - A).length();
  }

  Vec2<T> &A() { return A_; }
  Vec2<T> &B() { return B_; }
  Vec2<T> &C() { return C_; }

  T &a() { return a_; }
  T &b() { return b_; }
  T &c() { return c_; }

  T Area() {
    T s = a_ + b_ + c_;
    return sqrt(s * (s - a_) * (s - b_) * (s - c_));
  }

  bool static IsInside(Vec2<T> pt, Triangle<T> tri) {
    T a0 = tri.Area();
    T a1 = Triangle(pt, tri.B(), tri.C()).Area();
    T a2 = Triangle(pt, tri.A(), tri.C()).Area();
    T a3 = Triangle(pt, tri.A(), tri.B()).Area();
    return (a0 = a1 + a2 + a3);
  }
};

} // namespace common_utilities
