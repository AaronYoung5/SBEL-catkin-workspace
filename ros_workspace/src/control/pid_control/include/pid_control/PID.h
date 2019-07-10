// ROS include
#include "ros/ros.h"

// Message type includes
#include "common_msgs/ConeMap.h"
#include "common_msgs/Control.h"
#include "common_msgs/VehState.h"
#include "geometry_msgs/Point.h"

// Service type includes
#include "common_srvs/ConeMap.h"
#include "common_srvs/Path.h"

// Utilities
#include "common_utilities/Vector.h"

// Namespace declarations
using namespace common_utilities;

class PID {
private:
  // Private variables

  // PID Settings
  float kp_, ki_, kd_;                // PID tuning constants
  float p_error_, d_error_, i_error_; // PID errors

public:
  // Public methods

  // Constructor
  PID(float kp, float ki, float kd);

private:
  // Private methods

  // PID computation methods
  // Computes and returns the total error
  float totalError();
  // Updates the error using the cross track error (cte)
  void updateError(float cte);
};
