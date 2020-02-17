#pragma once

#include "synchrono_ros/flatbuffers/messages/SynMessage.h"

class  SynSPATMessage : public SynMessage {
  public:
    enum Color { GREEN, YELLOW, RED };

    struct Lane {
        Color color;
    };

    struct State : SynMessage::State {
        double sender;
        double intersection;
        std::vector<Lane> lanes;

        State() : SynMessage::State(0.0) {}

        State(double time, double sender, double intersection, std::vector<Lane> lanes)
            : SynMessage::State(time), sender(sender), intersection(intersection), lanes(lanes) {}
    };

    SynSPATMessage(int id) : SynMessage(id, SynMessage::SPAT) {}
    SynSPATMessage(int id, std::shared_ptr<SynMessage::State> state) : SynMessage(id, SynMessage::SPAT) {
        m_state = std::static_pointer_cast<State>(state);
    }
    SynSPATMessage(int id, const SynFlatBuffers::Message* message) : SynMessage(id, SynMessage::SPAT) {
        GenerateState(message);
    }

    virtual void GenerateState(const SynFlatBuffers::Message* message) {
        const SynFlatBuffers::Pointer* pointer = static_cast<const SynFlatBuffers::Pointer*>(message->message());
        const SynFlatBuffers::Buffer* buffer = flatbuffers::GetRoot<SynFlatBuffers::Buffer>(pointer->pointer()->Data());
        const SynFlatBuffers::Message* msg = buffer->buffer()->Get(0);
        const SynFlatBuffers::SPAT::State* state = msg->message_as_SPAT_State();
        std::vector<Lane> lanes;
        for (auto lane : (*state->lanes())) {
            lanes.push_back(Lane({static_cast<Color>(lane->color())}));
        }
        m_state = std::make_shared<State>(state->time(), state->sender(), state->intersection(), lanes);
    }

    State GenerateState(const SynFlatBuffers::SPAT::State* state) {
        std::vector<Lane> lanes;
        for (auto lane : (*state->lanes())) {
            lanes.push_back(Lane({static_cast<Color>(lane->color())}));
        }

        return State(state->time(), state->sender(), state->intersection(), lanes);
    }

    flatbuffers::Offset<SynFlatBuffers::SPAT::State> GenerateMessage(flatbuffers::FlatBufferBuilder& builder,
                                                                     std::shared_ptr<SynSPATMessage::State> state) {
        std::vector<flatbuffers::Offset<SynFlatBuffers::SPAT::Lane>> lanes;
        for (Lane lane : state->lanes) {
            auto color = SynFlatBuffers::SPAT::Color(lane.color);
            lanes.push_back(SynFlatBuffers::SPAT::CreateLane(builder, static_cast<SynFlatBuffers::SPAT::Color>(color)));
        }

        flatbuffers::Offset<SynFlatBuffers::SPAT::State> flatbuffer_state = SynFlatBuffers::SPAT::CreateState(
            builder, state->time, state->sender, state->intersection, builder.CreateVector(lanes));
        return flatbuffer_state;
    }

    virtual FlatBufferMessage GenerateMessage(flatbuffers::FlatBufferBuilder& builder) {
        std::shared_ptr<State> state = std::static_pointer_cast<State>(m_state);

        std::vector<flatbuffers::Offset<SynFlatBuffers::SPAT::Lane>> lanes;
        for (Lane lane : state->lanes) {
            auto color = SynFlatBuffers::SPAT::Color(lane.color);
            lanes.push_back(SynFlatBuffers::SPAT::CreateLane(builder, static_cast<SynFlatBuffers::SPAT::Color>(color)));
        }

        flatbuffers::Offset<SynFlatBuffers::SPAT::State> flatbuffer_state = SynFlatBuffers::SPAT::CreateState(
            builder, state->time, state->sender, state->intersection, builder.CreateVector(lanes));

        FlatBufferMessage message = flatbuffers::Offset<SynFlatBuffers::Message>(
            SynFlatBuffers::CreateMessage(builder, SynFlatBuffers::Type_SPAT_State, flatbuffer_state.Union(),
                                          builder.CreateString(std::to_string(m_id))));
        return message;
    }

  private:
};
