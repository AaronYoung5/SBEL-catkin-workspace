#pragma once

#include "synchrono_interface/SynComponent.h"

#include "chrono_sensor/ChCameraSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"

#include "synchrono/flatbuffers/messages/SynCameraMessage.h"

using namespace chrono::sensor;

namespace synchrono {
namespace interface {

static int num_cameras = 0;

class SynCameraComponent : public SynComponent {
  private:
    std::shared_ptr<ChCameraSensor> m_camera;

  public:
    SynCameraComponent(std::shared_ptr<ChCameraSensor> camera)
        : SynComponent(SynComponent::SENDER, camera->GetName(), camera->GetUpdateRate()), m_camera(camera) {
        m_msg = std::make_shared<SynCameraMessage>(num_cameras++);
    }

    void Advance(double time) {
        std::shared_ptr<SynCameraMessage::State> state =
            std::static_pointer_cast<SynCameraMessage::State>(m_msg->GetState());

        UserR8BufferPtr buffer_ptr = m_camera->GetMostRecentBuffer<UserR8BufferPtr>();
        state->height = buffer_ptr->Height;
        state->width = buffer_ptr->Width;
        state->bytes_per_pixel = sizeof(PixelRGBA8);
        int size = state->height * state->width * state->bytes_per_pixel;
        state->data.resize(size);
        memcpy(state->data.data(), buffer_ptr->Buffer.get(), size);
    }
};
}  // namespace interface
}  // namespace synchrono
