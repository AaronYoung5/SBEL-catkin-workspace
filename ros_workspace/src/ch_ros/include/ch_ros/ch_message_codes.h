#pragma once

namespace ChMessageCodes {
  enum ChMessageCode {
    LIDAR,
    GPS,
    IMU,
    VEHICLE,
    CONTROL,
    TIME,
    LIGHT,
    CONE,
    EXIT
  };
}
typedef ChMessageCodes::ChMessageCode ChMessageCode;
