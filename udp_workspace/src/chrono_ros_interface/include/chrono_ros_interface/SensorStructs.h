#pragma once

#include "sensor_msgs/NavSatFix.h"

struct Vector3D {
        float x;
        float y;
        float z;

        Vector3D operator- (Vector3D &v2) { return Vector3D { x - v2.x, y - v2.y, z - v2.z }; }

        float static Length(Vector3D v) {
                return sqrt(pow(v.x, 2) + pow(v.y,2) + pow(v.z,2));
        }

        float dot(Vector3D &v2) { return x * v2.x + y * v2.y + z *v2.z; }
};

struct Quaternion {
        float x;
        float y;
        float z;
        float w;
};

struct Location {
        float altitude = 0;
        float longitude = 0;
        float latitude = 0;

        Location static NavSatFix_To_Location(const sensor_msgs::NavSatFix::ConstPtr gps) {
                Location loc;
                loc.altitude = (float)gps->altitude;
                loc.longitude = (float)gps->longitude;
                loc.latitude = (float)gps->latitude;
                return loc;
        }
};

struct IMU {
        Vector3D linear_acceleration;
        Vector3D angular_velocity;
        Quaternion orientation;
};

struct Light {
        float xPos;
        float yPos;
        int xDir;
        int yDir;
        float offset;
};
