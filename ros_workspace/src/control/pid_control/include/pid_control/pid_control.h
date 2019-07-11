// ROS include
#include "ros/ros.h"

// Package includes
#include "pid_control/PID.h"

// Message type includes
#include "common_msgs/ConeMap.h"
#include "common_msgs/Control.h"
#include "common_msgs/VehState.h"
#include "common_msgs/PIDDebug.h"

// Service type includes
#include "common_srvs/ConeMap.h"
#include "common_srvs/Path.h"

// Utilities
#include "common_utilities/Vector.h"

// Namespace declarations
using namespace common_utilities;

class PIDControl {
private:
  // Private Variables

  // PID Speed Controller
  PID speed_controller_;
  // PID Steering Controller
  PID steering_controller_;

  // Controls publisher
  ros::Publisher controls_pub_;
  // Publisher for debugging purposes
  ros::Publisher debug_pub_;

  // Cone map subscriber
  // ros::Subscriber cone_sub_;
  // Vehicle state subscriber
  ros::Subscriber state_sub_;

  // Path service server
  ros::ServiceServer path_srv_;

  // Cone service client
  ros::ServiceClient cone_client_;

  // Controls values
  float throttle_, braking_, steering_;

  // Path vector
  std::vector<Vector3D<>> path_;

public:
  // Public Methods
  PIDControl(ros::NodeHandle n);

private:
  // Private Methods

  // Initialization of class
  void initialize();

  // Publish controls message
  void publishControls();

  // Publish debug message
  void publishDebug(Vector3D<> pos, Vector3D<> begin, Vector3D<> end);

  // Cone map subscriber callback
  void coneCallback(const common_msgs::ConeMap::ConstPtr &msg);
  // Vehicle state subscriber callback
  void stateCallback(const common_msgs::VehState::ConstPtr &msg);

  // Path service callback
  bool pathCallback(common_srvs::Path::Request &req,
                    common_srvs::Path::Response &res);

  // Clamping function for the control values
  void clampControls();

  // Computes the minimum distance from a point to a line segment
  float minimumDistance(Vector3D<> pos, Vector3D<> begin, Vector3D<> end);
};
