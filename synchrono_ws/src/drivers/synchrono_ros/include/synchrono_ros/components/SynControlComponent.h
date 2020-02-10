#include "synchrono_interface/SynComponent.h"

#include "synchrono_interface/SynInterfaceDriver.h"

#include "synchrono/flatbuffers/messages/SynControlMessage.h"

namespace synchrono {
namespace interface {

class SynControlComponent : public SynComponent {
  private:
    SynInterfaceDriver& m_driver;

  public:
    SynControlComponent(std::string id, SynInterfaceDriver& driver)
        : SynComponent(SynComponent::RECEIVER, id), m_driver(driver) {}

    void Advance(double time) {
        std::shared_ptr<SynControlMessage::State> state =
            std::static_pointer_cast<SynControlMessage::State>(m_msg->GetState());
        m_driver.SetTargetThrottle(state->throttle);
        m_driver.SetTargetSteering(state->steering);
        m_driver.SetTargetBraking(state->braking);
    }
};
}  // namespace interface
}  // namespace synchrono
