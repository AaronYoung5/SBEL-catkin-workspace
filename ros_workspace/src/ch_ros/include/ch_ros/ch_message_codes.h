#pragma once

namespace ChMessageCodes {
  enum ChMessageCode {
    LIDAR,
    GPS,
    IMU,
    CONTROL,
    TIME,
    LIGHT,
    CONE,
    EXIT
  };
}
typedef ChMessageCodes::ChMessageCode ChMessageCode;
