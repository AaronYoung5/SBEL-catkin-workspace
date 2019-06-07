#include <chrono_ros_interface/SensorStructs.h>

class PIDFollower {
private:

public:

        float static Steering(std::vector<Vector3D> path, int seekIndex, Vector3D pos, Quaternion orientation);
};
