#include "kalman_filter/kalman_filter.h"

#include "common_utilities/GPS.h"

#include <Eigen/Dense>

class KalmanEstimation {
private:
  KalmanFilter<float, 3, 3> *filter;

  float dt = 1;

public:
  KalmanEstimation();
  ~KalmanEstimation();

private:
};
