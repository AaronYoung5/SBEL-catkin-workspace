#pragma once

namespace chrono {
namespace transport {
namespace TransportTypes {
enum TransportType { CAMERA, LIDAR, GPS, IMU, CONTROL, TIME, EXIT };
}
typedef TransportTypes::TransportType TransportType;
} // namespace transport
} // namespace chrono
