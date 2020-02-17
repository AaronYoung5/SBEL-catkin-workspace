#pragma once

#include "synchrono_ros/components/SynComponent.h"

#include "synchrono_ros/flatbuffers/messages/SynCameraMessage.h"

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

static int num_cameras = 0;

class SynCameraComponent : public SynComponent {
private:
  image_transport::Publisher pub_;

public:
  SynCameraComponent(ros::NodeHandle &n, std::string id)
      : SynComponent(SynComponent::RECEIVER, id) {
    image_transport::ImageTransport it(n);
    pub_ = it.advertise(id, 1);
    std::make_shared<SynCameraMessage>(num_cameras++);
  }

  void Advance(double time) {
    std::shared_ptr<SynCameraMessage::State> state =
        std::static_pointer_cast<SynCameraMessage::State>(m_msg->GetState());

    sensor_msgs::Image msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";

    msg.height = state->height;
    msg.width = state->width;

    msg.encoding = sensor_msgs::image_encodings::RGBA8;

    int num = 1; // for endianness detection
    msg.is_bigendian = !(*(char *)&num == 1);

    msg.step = state->bytes_per_pixel * msg.width;

    size_t size = msg.height * msg.step;
    msg.data.resize(size);
    memcpy((uint8_t *)(&msg.data[0]), state->data.data(), size);

    pub_.publish(msg);
  }
};
