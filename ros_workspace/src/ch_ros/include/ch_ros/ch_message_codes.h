#pragma once

namespace ChMessageCodes {
  enum ChMessageCode {
    LIDAR,
    GPS,
    IMU,
    CONTROL,
    TIME,
    VEHICLE,
    CONE,
    EXIT,
    CONFIG
  };
}
typedef ChMessageCodes::ChMessageCode ChMessageCode;
