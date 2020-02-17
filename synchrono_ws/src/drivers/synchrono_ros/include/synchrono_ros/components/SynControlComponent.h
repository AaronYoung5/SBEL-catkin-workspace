#include "synchrono_ros/components/SynComponent.h"

#include "synchrono_ros/flatbuffers/messages/SynControlMessage.h"

#include <common_msgs/Control.h>

class SynControlComponent : public SynComponent {
public:
  SynControlComponent(ros::NodeHandle &n, std::string id)
      : SynComponent(SynComponent::SENDER, id),
        sub_(n.subscribe(id, 1, &SynControlComponent::callback, this)) {}

  void Advance(double time) {
    std::shared_ptr<SynControlMessage::State> state =
        std::static_pointer_cast<SynControlMessage::State>(m_msg->GetState());

    state->throttle = driver_inputs_.throttle;
    state->steering = driver_inputs_.steering;
    state->braking = driver_inputs_.braking;
  }

  void callback(const common_msgs::Control::ConstPtr &msg) {
    driver_inputs_.throttle = msg->throttle;
    driver_inputs_.braking = msg->braking;
    driver_inputs_.steering = msg->steering;
  }

private:
  ros::Subscriber sub_;

  struct DriverInputs {
    float throttle = 0;
    float braking = 0;
    float steering = 0;
  } driver_inputs_;
};
