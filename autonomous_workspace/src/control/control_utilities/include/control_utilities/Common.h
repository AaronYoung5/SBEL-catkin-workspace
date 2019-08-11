#pragma once

#include <common_msgs/Control.h>

namespace control_utilities {
inline void clamp(common_msgs::Control &control) {
  control.throttle =
      control.throttle > 1 ? 1 : control.throttle < 0 ? 0 : control.throttle;
  control.steering =
      control.steering > 1 ? 1 : control.steering < -1 ? -1 : control.steering;
  control.braking =
      control.braking > 1 ? 1 : control.braking < 0 ? 0 : control.braking;
}
} // namespace control_utilities
