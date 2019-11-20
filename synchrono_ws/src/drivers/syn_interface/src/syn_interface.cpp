#include "syn_interface/syn_interface.h"

SynInterface::SynInterface(ros::NodeHandle &n) {
  Add(std::make_shared<SynCameraComponent>(n, "camera_sensor_1"));
  Add(std::make_shared<SynCameraComponent>(n, "camera_sensor_2"));
  Add(std::make_shared<SynTimeComponent>(n, "Time"));
  Add(std::make_shared<SynControlComponent>(n, "Control", "/control"));
  // Add(std::make_shared<SynLidarComponent>(n, "lidar"));

  std::string hostname, port;
  n.param<std::string>("hostname", hostname, "localhost");
  n.param<std::string>("port", port, "8080");

  transport_manager_.Connect(hostname, port);
}

void SynInterface::Run() {
  while (ros::ok()) {
    for (auto component : components_) {
      if (component->GetType() == SynInterfaceComponent::Sender) {
        flatbuffers_manager_.AddMessage(
            component->toFlatBuffer(flatbuffers_manager_.GetBuilder()));
      }
    }

    transport_manager_.Receive(flatbuffers_manager_);
    transport_manager_.Send(flatbuffers_manager_);

    for (auto component : components_) {
      if (component->GetType() == SynInterfaceComponent::Receiver) {
        if (flatbuffers_manager_.HasMessage(component->GetName())) {
          component->fromFlatBuffer(
              flatbuffers_manager_.GetMessage(component->GetName()));
        }
      }
    }
    flatbuffers_manager_.Reset();
    ros::spinOnce();
  }
}

void SynInterface::Add(std::shared_ptr<SynInterfaceComponent> component) {
  auto it = std::find(names_.begin(), names_.end(), component->GetName());
  if (it != names_.end()) {
    ROS_FATAL("Each InterfaceComponent must have a unique name.");
    ros::shutdown();
    exit(-1);
  }

  components_.push_back(component);
  names_.push_back(component->GetName());
}
