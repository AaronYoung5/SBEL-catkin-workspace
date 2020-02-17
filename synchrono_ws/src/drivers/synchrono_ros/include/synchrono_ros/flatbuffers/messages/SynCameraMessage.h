#pragma once

#include "synchrono_ros/flatbuffers/messages/SynMessage.h"

class  SynCameraMessage : public SynMessage {
  public:
    struct State : SynMessage::State {
        int height;
        int width;
        int bytes_per_pixel;
        std::vector<uint8_t> data;

        State() : SynMessage::State(0.0), height(0), width(0), bytes_per_pixel(0) {}

        State(double time, int height, int width, int bytes_per_pixel, std::vector<uint8_t> data)
            : SynMessage::State(time), height(height), width(width), bytes_per_pixel(bytes_per_pixel), data(data) {}
    };

    SynCameraMessage(int id) : SynMessage(id, SynMessage::CAMERA) { m_state = std::make_shared<State>(State()); }

    SynCameraMessage(int id, std::shared_ptr<SynMessage::State> state) : SynMessage(id, SynMessage::CAMERA) {
        m_state = std::static_pointer_cast<State>(state);
    }
    SynCameraMessage(int id, const SynFlatBuffers::Message* message) : SynMessage(id, SynMessage::CAMERA) {
        GenerateState(message);
    }

    virtual void GenerateState(const SynFlatBuffers::Message* message) {
        const SynFlatBuffers::Pointer* pointer = static_cast<const SynFlatBuffers::Pointer*>(message->message());
        const SynFlatBuffers::Buffer* buffer = flatbuffers::GetRoot<SynFlatBuffers::Buffer>(pointer->pointer()->Data());
        const SynFlatBuffers::Message* msg = buffer->buffer()->Get(0);
        const SynFlatBuffers::Interface::State* state = msg->message_as_Interface_State();
        const SynFlatBuffers::Interface::Camera* camera = state->state_as_Camera();
        int size = camera->height() * camera->width() * camera->bytes_per_pixel();
        std::vector<uint8_t> data(size);
        std::memcpy(data.data(), camera->data(), size);
        m_state = std::make_shared<State>(state->time()->t(), camera->height(), camera->width(),
                                          camera->bytes_per_pixel(), data);
    }

    virtual FlatBufferMessage GenerateMessage(flatbuffers::FlatBufferBuilder& builder) {
        std::shared_ptr<State> state = std::static_pointer_cast<State>(m_state);

        flatbuffers::Offset<SynFlatBuffers::Interface::Camera> camera = SynFlatBuffers::Interface::CreateCamera(
            builder, state->height, state->width, state->bytes_per_pixel,
            builder.CreateVector<unsigned char>(state->data.data(), state->data.size()));

        flatbuffers::Offset<SynFlatBuffers::Interface::State> flatbuffers_state =
            SynFlatBuffers::Interface::CreateState(builder, SynFlatBuffers::Interface::Type_Camera, camera.Union(),
                                                   state->time);

        FlatBufferMessage message = flatbuffers::Offset<SynFlatBuffers::Message>(
            SynFlatBuffers::CreateMessage(builder, SynFlatBuffers::Type_Interface_State, flatbuffers_state.Union(),
                                          builder.CreateString(std::to_string(m_id))));

        return message;
    }

  private:
};
