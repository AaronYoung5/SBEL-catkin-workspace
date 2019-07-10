// Header Include
#include "pid_control/pid_control.h"

PIDControl::PIDControl(ros::NodeHandle n)
    : controls_pub_(n.advertise<common_msgs::Control>("control", 10)),
      point_following_pub_(
          n.advertise<geometry_msgs::Point>("point_following", 10)),
      state_sub_(
          n.subscribe<common_msgs::VehState>("vehicle", 10, stateCallback)),
      path_srv_(n.advertiseService<common_srvs::Path>(
          "path", &PIDControl::pathCallback, this)),
      cone_client_(n.serviceClient<common_srvs::Path>("path", true)),
      steering_controller_(.8, 0, 0) {
  initialize();
}

void PIDControl::initialize() {
  common_srvs::ConeMap cone_srv;
  while (!client_.call(cone_srv)) {
  }
  std::cout << "Cones Received" << std::endl;
  for (int i = sizeof(cone_srv.response.blue_cones) / 2 - 1; i >= 0; i--) {
    // printPoint("Blue Cone", cone_srv.response.blue_cones[i].position.x,
    //            cone_srv.response.blue_cones[i].position.y,
    //            cone_srv.response.blue_cones[i].position.z);
    // printPoint("Yellow Cone", cone_srv.response.yellow_cones[i].position.x,
    //            cone_srv.response.yellow_cones[i].position.y,
    //            cone_srv.response.yellow_cones[i].position.z);
    // printPoint("Path Position", (cone_srv.response.blue_cones[i].position.x +
    //  cone_srv.response.yellow_cones[i].position.x) /
    //     2,
    // (cone_srv.response.blue_cones[i].position.y +
    //  cone_srv.response.yellow_cones[i].position.y) /
    //     2,
    // (cone_srv.response.blue_cones[i].position.z +
    //  cone_srv.response.yellow_cones[i].position.z) /
    //     2);
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
