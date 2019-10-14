#include "syn_interface/components/syn_interface_component.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <image_transport/image_transport.h>

class SynCameraComponent : public SynInterfaceComponent {
private:
  image_transport::Publisher pub_;

public:
  SynCameraComponent(ros::NodeHandle &n, std::string name)
      : SynInterfaceComponent(SynComponentType::Receiver, name),
        pub_(image_transport::ImageTransport(n).advertise(name, 1)) {}

  std::shared_ptr<flatbuffers::Offset<SynInterfaceMessage::Message>> toFlatBuffer(flatbuffers::FlatBufferBuilder &builder_) {}

  void fromFlatBuffer(const SynInterfaceMessage::Message *message) {
    const SynInterfaceMessage::Camera *camera =
        static_cast<const SynInterfaceMessage::Camera *>(message->message());

    sensor_msgs::Image msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";

    msg.height = camera->height();
    msg.width = camera->width();

    msg.encoding = sensor_msgs::image_encodings::RGBA8;

    int num = 1; // for endianness detection
    msg.is_bigendian = !(*(char *)&num == 1);

    msg.step = camera->bytes_per_pixel() * msg.width;

    size_t size = msg.width * msg.height * camera->bytes_per_pixel();
    msg.data.resize(size);
    memcpy((uint8_t *)(&msg.data[0]), camera->data()->Data(), size);

    pub_.publish(msg);
  }
};
