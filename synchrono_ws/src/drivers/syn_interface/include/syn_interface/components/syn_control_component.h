#include "syn_interface/components/syn_interface_component.h"

#include <common_msgs/Control.h>

class SynControlComponent : public SynInterfaceComponent {
private:
  ros::Subscriber sub_;

  struct DriverInputs {
    float throttle = 0;
    float braking = 0;
    float steering = 0;
  } driver_inputs_;

public:
  SynControlComponent(ros::NodeHandle &n, std::string name, std::string topic)
      : SynInterfaceComponent(SynComponentType::Sender, name),
        sub_(n.subscribe(topic, 1, &SynControlComponent::controlCallback,
                         this)) {}

  void controlCallback(const common_msgs::Control::ConstPtr &msg) {
    driver_inputs_.throttle = msg->throttle;
    driver_inputs_.braking = msg->braking;
    driver_inputs_.steering = msg->steering;
  }

  std::shared_ptr<flatbuffers::Offset<SynInterfaceMessage::Message>>
  toFlatBuffer(flatbuffers::FlatBufferBuilder &builder) {
    flatbuffers::Offset<SynInterfaceMessage::Control> control =
        SynInterfaceMessage::CreateControl(builder, driver_inputs_.throttle,
                                           driver_inputs_.steering,
                                           driver_inputs_.braking);
    std::shared_ptr<flatbuffers::Offset<SynInterfaceMessage::Message>>
        control_message =
            std::make_shared<flatbuffers::Offset<SynInterfaceMessage::Message>>(
                SynInterfaceMessage::CreateMessage(
                    builder, SynInterfaceMessage::Type_Control, control.Union(),
                    builder.CreateString(name_)));
    return control_message;
  }

  void fromFlatBuffer(const SynInterfaceMessage::Message *message) {}
};
