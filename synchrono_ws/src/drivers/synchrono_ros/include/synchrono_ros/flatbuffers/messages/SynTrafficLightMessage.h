#pragma once

#include "synchrono_ros/flatbuffers/messages/SynMessage.h"
#include "SynSPATMessage.h"
#include "SynMAPMessage.h"

class  SynTrafficLightMessage : public SynMessage {
  public:
    struct State : SynMessage::State {
        SynMAPMessage::State map;
        SynSPATMessage::State spat;

        State() : SynMessage::State(0.0), map(SynMAPMessage::State()), spat(SynSPATMessage::State()) {}

        State(double time, SynMAPMessage::State map, SynSPATMessage::State spat)
            : SynMessage::State(time), map(map), spat(spat) {}
    };

    SynTrafficLightMessage(int id)
        : SynMessage(id, SynMessage::TRAFFIC_LIGHT),
          m_map_msg(std::make_shared<SynMAPMessage>(id)),
          m_spat_msg(std::make_shared<SynSPATMessage>(id)) {
        m_state = std::make_shared<State>(State());
    }

    SynTrafficLightMessage(int id, std::shared_ptr<SynMessage::State> state)
        : SynMessage(id, SynMessage::TRAFFIC_LIGHT) {
        m_state = std::static_pointer_cast<State>(state);
        m_map_msg = std::make_shared<SynMAPMessage>(
            id, std::make_shared<SynMAPMessage::State>(std::static_pointer_cast<State>(m_state)->map));
        m_spat_msg = std::make_shared<SynSPATMessage>(
            id, std::make_shared<SynSPATMessage::State>(std::static_pointer_cast<State>(m_state)->spat));
    }

    SynTrafficLightMessage(int id, const SynFlatBuffers::Message* message) : SynMessage(id, SynMessage::TRAFFIC_LIGHT) {
        GenerateState(message);
        m_map_msg = std::make_shared<SynMAPMessage>(
            id, std::make_shared<SynMAPMessage::State>(std::static_pointer_cast<State>(m_state)->map));
        m_spat_msg = std::make_shared<SynSPATMessage>(
            id, std::make_shared<SynSPATMessage::State>(std::static_pointer_cast<State>(m_state)->spat));
    }

    virtual void GenerateState(const SynFlatBuffers::Message* message) {
        const SynFlatBuffers::Pointer* pointer = static_cast<const SynFlatBuffers::Pointer*>(message->message());
        const SynFlatBuffers::Buffer* buffer = flatbuffers::GetRoot<SynFlatBuffers::Buffer>(pointer->pointer()->Data());
        const SynFlatBuffers::Message* msg = buffer->buffer()->Get(0);
        const SynFlatBuffers::Agent::State* agent_state = msg->message_as_Agent_State();
        const SynFlatBuffers::Agent::TrafficLight::State* state = agent_state->message_as_TrafficLight_State();
        m_state = std::make_shared<State>(state->time(), m_map_msg->GenerateState(state->map()),
                                          m_spat_msg->GenerateState(state->spat()));
    }

    virtual FlatBufferMessage GenerateMessage(flatbuffers::FlatBufferBuilder& builder) {
        std::shared_ptr<State> state = std::static_pointer_cast<State>(m_state);

        flatbuffers::Offset<SynFlatBuffers::Agent::State> flatbuffer_state = SynFlatBuffers::Agent::CreateState(
            builder, SynFlatBuffers::Agent::Type_TrafficLight_State,
            SynFlatBuffers::Agent::TrafficLight::CreateState(
                builder, state->time,
                m_map_msg->GenerateMessage(
                    builder, std::make_shared<SynMAPMessage::State>(std::static_pointer_cast<State>(m_state)->map)),
                m_spat_msg->GenerateMessage(
                    builder, std::make_shared<SynSPATMessage::State>(std::static_pointer_cast<State>(m_state)->spat)))
                .Union());

        FlatBufferMessage message = flatbuffers::Offset<SynFlatBuffers::Message>(
            SynFlatBuffers::CreateMessage(builder, SynFlatBuffers::Type_Agent_State, flatbuffer_state.Union(),
                                          builder.CreateString(std::to_string(m_id))));
        return message;
    }

  private:
    std::shared_ptr<SynSPATMessage> m_spat_msg;
    std::shared_ptr<SynMAPMessage> m_map_msg;
};
