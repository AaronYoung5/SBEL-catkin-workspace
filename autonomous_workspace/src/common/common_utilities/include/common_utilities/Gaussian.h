#pragma once

#include <math.h>

namespace common_utilities {

class Gaussian {
private:
  float mu_;
  float sd_;
  float var_;

public:
  Gaussian(float mu, float var) : mu_(mu), var_(var) {}

  float mu() { return mu_; }
  float var() { return var_; }

  void setMu(float mu) { mu_ = mu; }
  void setVar(float var) { var_ = var; }

  Gaussian operator+(Gaussian &g) { return Gaussian::sum(*this, g); }

  Gaussian operator*(Gaussian &g) { return Gaussian::multiply(*this, g); }

  static Gaussian sum(Gaussian &g1, Gaussian &g2) {
    float mu = g1.mu() + g2.mu();
    float var = g1.var() + g2.var();
    return Gaussian(mu, var);
  }

  static Gaussian multiply(Gaussian &g1, Gaussian &g2) {
    float mu =
        (g1.var() * g2.mu() + g2.var() * g1.mu()) / (g1.var() + g2.var());
    float var = (g1.var() * g2.var()) / (g1.var() + g2.var());
    return Gaussian(mu, var);
  }
};
} // namespace common_utilities
