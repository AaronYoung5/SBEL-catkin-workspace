#include "syn_interface/components/syn_interface_component.h"

#include <sensor_msgs/PointCloud2.h>

class SynLidarComponent : public SynInterfaceComponent {
private:
  ros::Publisher pub_;

public:
  SynLidarComponent(ros::NodeHandle &n, std::string name)
      : SynInterfaceComponent(SynComponentType::Receiver, name),
        pub_(n.advertise<sensor_msgs::PointCloud2>(name, 1)) {}

  std::shared_ptr<flatbuffers::Offset<SynInterfaceMessage::Message>>
  toFlatBuffer(flatbuffers::FlatBufferBuilder &builder_) {}

  void fromFlatBuffer(const SynInterfaceMessage::Message *message) {
    const SynInterfaceMessage::Lidar *lidar =
        static_cast<const SynInterfaceMessage::Lidar *>(message->message());

    sensor_msgs::PointCloud2 msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";

    msg.width = lidar->data()->Length();
    msg.height = 1;

    // Convert x/y/z to fields
    msg.fields.resize(3);
    msg.fields[0].name = "x";
    msg.fields[1].name = "y";
    msg.fields[2].name = "z";

    int offset = 0;
    for (size_t d = 0; d < msg.fields.size(); d++, offset += 4) {
      msg.fields[d].offset = offset;
      msg.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
      msg.fields[d].count = 1;
    }

    msg.point_step = offset;
    msg.row_step = msg.point_step * msg.width;

    msg.data.resize(msg.row_step);
    msg.is_bigendian = false;
    msg.is_dense = false;

    memcpy(&msg.data, lidar->data()->Data(), lidar->data()->Length());

    pub_.publish(msg);
  }
};
