#include "synchrono_ros/SynROSInterface.h"

namespace synchrono {
namespace interface {
SynROSInterface::SynROSInterface(ros::NodeHandle &n, SynFlatBuffersManager& flatbuffers_manager)
    : SynInterface(flatbuffers_manager) {

      std::string hostname, port;
      n.param<std::string>("hostname", hostname, "localhost");
      n.param<std::string>("port", port, "8080");

      m_ros_transport_manager.Connect(hostname, port);
    }

void SynROSInterface::Synchronize(double ch_time) {
    for (auto component : m_components) {
        component.second->Advance(ch_time);
        if (component.second->GetType() == SynComponent::SENDER) {
            if (component.second->ShouldUpdate(ch_time)) {
                m_flatbuffers_manager.AddMessage(
                    component.second->GetMessage()->GenerateMessage(m_flatbuffers_manager.GetBuilder()));
            }
        }
    }

    m_transport_manager.Send(m_flatbuffers_manager);
    m_transport_manager.Receive(m_flatbuffers_manager);

    for (auto component : m_components) {
        if (component.second->GetType() == SynComponent::RECEIVER) {
            if (auto msg = m_flatbuffers_manager.Get(component.second->GetID())) {
                component.second->GetMessage()->GenerateState(msg);
            }
        }
    }

    m_flatbuffers_manager.Reset();
}

}  // namespace interface
}  // namespace synchrono
