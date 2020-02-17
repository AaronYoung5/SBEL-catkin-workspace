#pragma once

#include "synchrono_ros/flatbuffers/messages/SynMessage.h"

class  SynMAPMessage : public SynMessage {
  public:
    struct Lane {
        double width;
        ChVector<> stopPos;
        ChVector<> offsetPos;
    };

    struct State : SynMessage::State {
        double sender;
        double intersection;
        std::vector<Lane> lanes;

        State() : SynMessage::State(0.0) {}

        State(double time, double sender, double intersection, std::vector<Lane> lanes)
            : SynMessage::State(time), sender(sender), intersection(intersection), lanes(lanes) {}
    };

    SynMAPMessage(int id) : SynMessage(id, SynMessage::MAP) {}
    SynMAPMessage(int id, std::shared_ptr<SynMessage::State> state) : SynMessage(id, SynMessage::MAP) {
        m_state = std::static_pointer_cast<State>(state);
    }
    SynMAPMessage(int id, const SynFlatBuffers::Message* message) : SynMessage(id, SynMessage::MAP) {
        GenerateState(message);
    }

    virtual void GenerateState(const SynFlatBuffers::Message* message) {
        const SynFlatBuffers::Pointer* pointer = static_cast<const SynFlatBuffers::Pointer*>(message->message());
        const SynFlatBuffers::Buffer* buffer = flatbuffers::GetRoot<SynFlatBuffers::Buffer>(pointer->pointer()->Data());
        const SynFlatBuffers::Message* msg = buffer->buffer()->Get(0);
        const SynFlatBuffers::MAP::State* state = msg->message_as_MAP_State();
        std::vector<Lane> lanes;
        for (auto lane : (*state->lanes())) {
            lanes.push_back(Lane({lane->width(),
                                  {lane->stopPos()->x(), lane->stopPos()->y(), lane->stopPos()->z()},
                                  {lane->offsetPos()->x(), lane->offsetPos()->y(), lane->offsetPos()->z()}}));
        }
        this->m_state = std::make_shared<State>(state->time(), state->sender(), state->intersection(), lanes);
    }

    State GenerateState(const SynFlatBuffers::MAP::State* state) {
        std::vector<Lane> lanes;
        for (auto lane : (*state->lanes())) {
            lanes.push_back(Lane({lane->width(),
                                  {lane->stopPos()->x(), lane->stopPos()->y(), lane->stopPos()->z()},
                                  {lane->offsetPos()->x(), lane->offsetPos()->y(), lane->offsetPos()->z()}}));
        }
        return State(state->time(), state->sender(), state->intersection(), lanes);
    }

    flatbuffers::Offset<SynFlatBuffers::MAP::State> GenerateMessage(flatbuffers::FlatBufferBuilder& builder,
                                                                    std::shared_ptr<SynMAPMessage::State> state) {
        std::vector<flatbuffers::Offset<SynFlatBuffers::MAP::Lane>> lanes;
        for (Lane l : state->lanes) {
            lanes.push_back(SynFlatBuffers::MAP::CreateLane(
                builder, l.width,
                SynFlatBuffers::MAP::CreateVector(builder, l.stopPos.x(), l.stopPos.y(), l.stopPos.z()),
                SynFlatBuffers::MAP::CreateVector(builder, l.offsetPos.x(), l.offsetPos.y(), l.offsetPos.z())));
        }

        flatbuffers::Offset<SynFlatBuffers::MAP::State> flatbuffer_state = SynFlatBuffers::MAP::CreateState(
            builder, state->time, state->sender, state->intersection, builder.CreateVector(lanes));
        return flatbuffer_state;
    }

    virtual FlatBufferMessage GenerateMessage(flatbuffers::FlatBufferBuilder& builder) {
        std::shared_ptr<State> state = std::static_pointer_cast<State>(m_state);

        std::vector<flatbuffers::Offset<SynFlatBuffers::MAP::Lane>> lanes;
        for (Lane l : state->lanes) {
            lanes.push_back(SynFlatBuffers::MAP::CreateLane(
                builder, l.width,
                SynFlatBuffers::MAP::CreateVector(builder, l.stopPos.x(), l.stopPos.y(), l.stopPos.z()),
                SynFlatBuffers::MAP::CreateVector(builder, l.offsetPos.x(), l.offsetPos.y(), l.offsetPos.z())));
        }

        flatbuffers::Offset<SynFlatBuffers::MAP::State> flatbuffer_state = SynFlatBuffers::MAP::CreateState(
            builder, state->time, state->sender, state->intersection, builder.CreateVector(lanes));

        FlatBufferMessage message = flatbuffers::Offset<SynFlatBuffers::Message>(
            SynFlatBuffers::CreateMessage(builder, SynFlatBuffers::Type_MAP_State, flatbuffer_state.Union(),
                                          builder.CreateString(std::to_string(m_id))));
        return message;
    }

  private:
};
