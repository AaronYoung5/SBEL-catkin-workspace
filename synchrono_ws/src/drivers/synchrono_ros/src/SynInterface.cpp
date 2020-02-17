#include "synchrono_ros/SynInterface.h"

SynInterface::SynInterface(ros::NodeHandle &n)
    : m_transport_manager(m_flatbuffers_manager) {
  std::string hostname, port;
  n.param<std::string>("hostname", hostname, "localhost");
  n.param<std::string>("port", port, "8080");

  m_transport_manager.Connect(hostname, port);
}

void SynInterface::Synchronize() {
  m_transport_manager.Receive();

  for (auto component : m_components) {
    if (component.second->GetType() == SynComponent::RECEIVER) {
      if (auto msg = m_flatbuffers_manager.Get(component.second->GetID())) {
        component.second->GetMessage()->GenerateState(msg);
      }
    }
  }
  double ch_time = 0.0;

  for (auto component : m_components) {
    if (component.second->GetType() == SynComponent::SENDER) {
      if (component.second->ShouldUpdate(ch_time)) {
        component.second->Advance(ch_time);
        m_flatbuffers_manager.AddMessage(
            component.second->GetMessage()->GenerateMessage(
                m_flatbuffers_manager.GetBuilder()));
      }
    }
  }
  m_transport_manager.Send();

  m_flatbuffers_manager.Reset();
}

void SynInterface::AddComponent(std::shared_ptr<SynComponent> component) {
  auto it = m_components.find(component->GetID());
  if (it != m_components.end()) {
    throw "Each component must have a unique name.";
  }
  m_components.insert(std::pair<std::string, std::shared_ptr<SynComponent>>(
      component->GetID(), component));
}
