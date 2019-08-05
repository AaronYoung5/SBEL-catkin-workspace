#pragma once

#include "Vector.h"

#define EARTH_RADIUS 6371000.0
#define PI 3.14159

namespace common_utilities {
template <class T = float> class GPS {
private:
  T latitude_;
  T longitude_;
  T altitude_;

  // using the authalic radius of the earth
  // static float earth_radius = 6371000.0;
  // static float PI = 3.14159;

public:
  GPS(T longitude, T latitude, T altitude)
      : longitude_(longitude), latitude_(latitude), altitude_(altitude) {}

  T latitude() { return latitude_; }
  T longitude() { return longitude_; }
  T altitude() { return altitude_; }

  void setLatitude(T latitude) { latitude_ = latitude; }
  void setLongitude(T longitude) { longitude_ = longitude; }
  void setAltitude(T altitude) { altitude_ = altitude; }

  static GPS<T> toGPSCoords(common_utilities::Vector3D<T> cart, GPS<T> origin) {
    float lat = (cart.y() / EARTH_RADIUS) * 180.0 / PI + origin.y();
    float lon =
        (cart.x() / (EARTH_RADIUS * cos(lat * PI / 180.0))) * 180.0 / PI +
        origin.x();
    float alt = cart.z() + origin.z();

    // sanitize the new gps coordinates
    if (lat < -90.0) {
      // NOT A GOOD APPROXIMATION NEAR THE POLES ANYWAY
    } else if (lat > 90) {
      // NOT A GOOD APPROXIMATION NEAR THE POLES ANYWAY
    }

    if (lon < -180.0) {
      lon = lon + 360.0;
    } else if (lon > 180.0) {
      lon = lon - 360.0;
    }

    return GPS<T>(lon, lat, alt);
  }

  static Vector3D<T> toLocalCoords(GPS<T> gps, GPS<T> origin) {
    float x = (gps.longitude() - origin.longitude()) * PI * EARTH_RADIUS *
              cos(gps.latitude() * PI / 180) / 180;
    float y = (gps.latitude() - origin.latidude()) * PI * EARTH_RADIUS / 180;
    float z = gps.altitude() - origin.altitude();

    return Vector3D<T>(x, y, z);
  }
};
} // namespace common_utilities
