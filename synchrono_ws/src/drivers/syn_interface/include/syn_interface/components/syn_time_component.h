#include "syn_interface/components/syn_interface_component.h"

#include <rosgraph_msgs/Clock.h>

class SynTimeComponent : public SynInterfaceComponent {
private:
  ros::Publisher pub_;

public:
  SynTimeComponent(ros::NodeHandle &n, std::string name)
      : SynInterfaceComponent(SynComponentType::Receiver, name),
        pub_(n.advertise<rosgraph_msgs::Clock>(name, 1)) {}

  std::shared_ptr<flatbuffers::Offset<SynInterfaceMessage::Message>> toFlatBuffer(flatbuffers::FlatBufferBuilder &builder_) {}

  void fromFlatBuffer(const SynInterfaceMessage::Message *message) {
    const SynInterfaceMessage::Time *time =
        static_cast<const SynInterfaceMessage::Time *>(message->message());

    rosgraph_msgs::Clock msg;

    msg.clock = ros::Time(time->t());

    pub_.publish(msg);
  }
};
