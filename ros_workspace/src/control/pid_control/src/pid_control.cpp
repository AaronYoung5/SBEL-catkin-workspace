// Header Include
#include "pid_control/pid_control.h"

PIDControl::PIDControl(ros::NodeHandle n)
    : controls_pub_(n.advertise<common_msgs::Control>("control", 10)),
      debug_pub_(n.advertise<common_msgs::PIDDebug>("pid_debug", 10)),
      state_sub_(n.subscribe<common_msgs::VehState>(
          "vehicle", 10, &PIDControl::stateCallback, this)),
      path_srv_(n.advertiseService("path", &PIDControl::pathCallback, this)),
      cone_client_(n.serviceClient<common_srvs::ConeMap>("cone_map", true)),
      steering_controller_(.6, 0, 5), speed_controller_(.1, 0, 0) {
  initialize();
}

void PIDControl::initialize() {
  common_srvs::ConeMap cone_srv;
  while (!cone_client_.call(cone_srv)) {
  }
  std::cout << "Cones Received" << std::endl;
  for (int i = cone_srv.response.blue_cones.size() - 1; i >= 0; i--) {
    // std::cout << "Blue Cone :: (" << cone_srv.response.blue_cones[i].position.x
              // << ", " << cone_srv.response.blue_cones[i].position.y << ", "
              // << cone_srv.response.blue_cones[i].position.z << ")" << std::endl;
    path_.push_back(Vector3D<>((cone_srv.response.blue_cones[i].position.x +
                                cone_srv.response.yellow_cones[i].position.x) /
                                   2,
                               (cone_srv.response.blue_cones[i].position.y +
                                cone_srv.response.yellow_cones[i].position.y) /
                                   2,
                               (cone_srv.response.blue_cones[i].position.z +
                                cone_srv.response.yellow_cones[i].position.z) /
                                   2));
  }
}

void PIDControl::publishControls() {
  common_msgs::Control control;

  clampControls();
  control.throttle = throttle_;
  control.steering = steering_;
  control.braking = braking_;

  controls_pub_.publish(control);
}

void PIDControl::publishDebug(Vector3D<> pos, Vector3D<> begin,
                              Vector3D<> end) {
  // Computes projection and publishes it

  // Create debug msg
  common_msgs::PIDDebug debug;

  // Compute projection
  const float len = (begin - end).lengthSquared();
  // Case where begin == end
  if (len == 0.0) {
    debug.point_following.x = begin.x();
    debug.point_following.y = begin.y();
    debug.point_following.z = begin.z();
  }
  // Calc % projection is of line segment
  // Cut off at [0,1] to remain within ends of line segment
  const float t =
      std::max(0.0f, std::min(1.0f, (pos - begin).dot(end - begin) / len));
  // Get projection
  Vector3D<> proj = begin + (end - begin) * t;

  debug.point_following.x = proj.x();
  debug.point_following.y = proj.y();
  debug.point_following.z = proj.z();

  debug_pub_.publish(debug);
}

void PIDControl::stateCallback(const common_msgs::VehState::ConstPtr &msg) {
  Vector3D<> pos(msg->state.position.x, msg->state.position.y,
                 msg->state.position.z);

  float min_dist, min_index;
  for (int i = 0; i < path_.size() - 1; i++) {
    float temp_dist = minimumDistance(pos, path_[i], path_[i + 1]);
    if (abs(temp_dist) < abs(min_dist) || i == 0) {
      min_dist = temp_dist;
      min_index = i;
    }
  }
  steering_controller_.updateError(min_dist);
  speed_controller_.updateError(abs(min_dist) * -1);

  // Release the gas pedal if the deviation from the lane's center is high
  throttle_ = (1 - speed_controller_.totalError()) / 3;
  if (throttle_ < 0) {
    throttle_ = .05;
    // throttle_ = 0;
  } // else {
  braking_ = 0;
  // }
  steering_ = steering_controller_.totalError();

  // Debug
  // std::cout << "Speed Controller Error :: " << speed_controller_.totalError()
  // << std::endl;
  // std::cout << "Minimum Distance :: " << min_dist << std::endl;
  // std::cout << "Throttle :: " << throttle_ << std::endl;
  // std::cout << "Steering :: " << steering_ << std::endl;
  // std::cout << "Braking :: " << braking_ << std::endl;

  publishControls();
  publishDebug(pos, path_[min_index], path_[min_index + 1]);
}

bool PIDControl::pathCallback(common_srvs::Path::Request &req,
                              common_srvs::Path::Response &res) {
  for (Vector3D<> vec : path_) {
    geometry_msgs::Point p;
    p.x = vec.x();
    p.y = vec.y();
    p.z = vec.z();
    res.points.push_back(p);
  }
  return true;
}

void PIDControl::clampControls() {
  // Clamp controls at certain values
  // Throttle :: [0,1]
  throttle_ = throttle_ > 1 ? 1 : throttle_ < 0 ? 0 : throttle_;
  // Steering :: [-1, 1]
  steering_ = steering_ > 1 ? 1 : steering_ < -1 ? -1 : steering_;
  // Braking :: [0, 1]
  braking_ = braking_ > 1 ? 1 : braking_ < 0 ? 0 : braking_;
}

float PIDControl::minimumDistance(Vector3D<> pos, Vector3D<> begin,
                                  Vector3D<> end) {
  const float len = (begin - end).lengthSquared();
  // Case where begin == end
  if (len == 0.0)
    return (pos - begin).length();
  // Calc % projection is of line segment
  // Cut off at [0,1] to remain within ends of line segment
  const float t =
      std::max(0.0f, std::min(1.0f, (pos - begin).dot(end - begin) / len));
  // Get projection
  Vector3D<> proj = begin + (end - begin) * t;
  // Minimum distance from position to line segment
  float dist = (pos - proj).length();
  // Determine which side of line vehicle is
  int sign = std::signbit((proj.x() - begin.x()) * (pos.y() - begin.y()) -
                          (proj.y() - begin.y()) * (pos.x() - begin.x()));
  // Debug
  // std::cout << "Distance :: " << dist << std::endl;
  // Return distance neg or pos depending on side of line vehicle is
  return sign ? dist : -dist;
}

// float PIDControl::minimumDistance(Vector3D<> pos, Vector3D<> begin,
// Vector3D<> end,
//                            bool publish) {
//   Vector3D<> segment = end - begin;
//   Vector3D<> projection =
//       segment.getUnit() * (pos - begin).dot(segment.getUnit());
//   if (publish) {
//     pointFollowingPublish(projection + begin);
//     return (pos - begin).cross(projection + begin).z() > 0 ? -(pos -
//     projection).length()
//                                          : (pos - projection).length();
//   }
//   return (pos - projection).length();
// }
