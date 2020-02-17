#pragma once

#include "synchrono_ros/flatbuffers/messages/SynMessage.h"

class  SynControlMessage : public SynMessage {
  public:
    struct State : SynMessage::State {
        double throttle;
        double braking;
        double steering;

        State() : SynMessage::State(0.0), throttle(0.0), braking(0.0), steering(0.0) {}

        State(double time, int throttle, int braking, int steering)
            : SynMessage::State(time), throttle(throttle), braking(braking), steering(steering) {}
    };

    SynControlMessage(int id) : SynMessage(id, SynMessage::CONTROL) { m_state = std::make_shared<State>(State()); }

    SynControlMessage(int id, std::shared_ptr<SynMessage::State> state) : SynMessage(id, SynMessage::CONTROL) {
        m_state = std::static_pointer_cast<State>(state);
    }
    SynControlMessage(int id, const SynFlatBuffers::Message* message) : SynMessage(id, SynMessage::CONTROL) {
        GenerateState(message);
    }

    virtual void GenerateState(const SynFlatBuffers::Message* message) {
        const SynFlatBuffers::Pointer* pointer = static_cast<const SynFlatBuffers::Pointer*>(message->message());
        const SynFlatBuffers::Buffer* buffer = flatbuffers::GetRoot<SynFlatBuffers::Buffer>(pointer->pointer()->Data());
        const SynFlatBuffers::Message* msg = buffer->buffer()->Get(0);
        const SynFlatBuffers::Interface::State* state = msg->message_as_Interface_State();
        const SynFlatBuffers::Interface::Control* control = state->state_as_Control();
        m_state =
            std::make_shared<State>(state->time()->t(), control->throttle(), control->braking(), control->steering());
    }

    virtual FlatBufferMessage GenerateMessage(flatbuffers::FlatBufferBuilder& builder) {
        std::shared_ptr<State> state = std::static_pointer_cast<State>(m_state);

        flatbuffers::Offset<SynFlatBuffers::Interface::Control> control =
            SynFlatBuffers::Interface::CreateControl(builder, state->throttle, state->braking, state->steering);

        flatbuffers::Offset<SynFlatBuffers::Interface::State> flatbuffers_state =
            SynFlatBuffers::Interface::CreateState(builder, SynFlatBuffers::Interface::Type_Control, control.Union(),
                                                   state->time);

        FlatBufferMessage message = flatbuffers::Offset<SynFlatBuffers::Message>(
            SynFlatBuffers::CreateMessage(builder, SynFlatBuffers::Type_Interface_State, flatbuffers_state.Union(),
                                          builder.CreateString(std::to_string(m_id))));

        return message;
    }

  private:
};
