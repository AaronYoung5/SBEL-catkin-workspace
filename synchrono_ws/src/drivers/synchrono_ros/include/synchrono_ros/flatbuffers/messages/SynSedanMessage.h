#pragma once

#include "synchrono_ros/flatbuffers/messages/SynMessage.h"

class  SynSedanMessage : public SynMessage {
  public:
    struct State : SynMessage::State {
        SynMessage::Pose chassis;

        SynMessage::Pose wheel0;
        SynMessage::Pose wheel1;
        SynMessage::Pose wheel2;
        SynMessage::Pose wheel3;

        State() : SynMessage::State(0.0) {}

        State(double time,
              SynMessage::Pose chassis,
              SynMessage::Pose wheel0,
              SynMessage::Pose wheel1,
              SynMessage::Pose wheel2,
              SynMessage::Pose wheel3)
            : SynMessage::State(time),
              chassis(chassis),
              wheel0(wheel0),
              wheel1(wheel1),
              wheel2(wheel2),
              wheel3(wheel3) {}
    };

    SynSedanMessage(int id) : SynMessage(id, SynMessage::VEH) {}

    SynSedanMessage(int id, std::shared_ptr<SynMessage::State> state) : SynMessage(id, SynMessage::VEH) {
        m_state = std::static_pointer_cast<State>(state);
    }

    SynSedanMessage(int id, const SynFlatBuffers::Message* message) : SynMessage(id, SynMessage::VEH) {
        GenerateState(message);
    }

    virtual void GenerateState(const SynFlatBuffers::Message* message) {
        const SynFlatBuffers::Pointer* pointer = static_cast<const SynFlatBuffers::Pointer*>(message->message());
        const SynFlatBuffers::Buffer* buffer = flatbuffers::GetRoot<SynFlatBuffers::Buffer>(pointer->pointer()->Data());
        const SynFlatBuffers::Message* msg = buffer->buffer()->Get(0);
        const SynFlatBuffers::Agent::State* agent_state = msg->message_as_Agent_State();
        const SynFlatBuffers::Agent::Sedan::State* state = agent_state->message_as_Sedan_State();
        m_state = std::make_shared<State>(
            state->time(),
            SynMessage::Pose(
                {ChVector<>({state->chassis()->pos()->x(), state->chassis()->pos()->y(), state->chassis()->pos()->z()}),
                 ChQuaternion<>({state->chassis()->rot()->e0(), state->chassis()->rot()->e1(),
                                 state->chassis()->rot()->e2(), state->chassis()->rot()->e3()})}),
            SynMessage::Pose(
                {ChVector<>({state->wheel0()->pos()->x(), state->wheel0()->pos()->y(), state->wheel0()->pos()->z()}),
                 ChQuaternion<>({state->wheel0()->rot()->e0(), state->wheel0()->rot()->e1(),
                                 state->wheel0()->rot()->e2(), state->wheel0()->rot()->e3()})}),
            SynMessage::Pose(
                {ChVector<>({state->wheel1()->pos()->x(), state->wheel1()->pos()->y(), state->wheel1()->pos()->z()}),
                 ChQuaternion<>({state->wheel1()->rot()->e0(), state->wheel1()->rot()->e1(),
                                 state->wheel1()->rot()->e2(), state->wheel1()->rot()->e3()})}),
            SynMessage::Pose(
                {ChVector<>({state->wheel2()->pos()->x(), state->wheel2()->pos()->y(), state->wheel2()->pos()->z()}),
                 ChQuaternion<>({state->wheel2()->rot()->e0(), state->wheel2()->rot()->e1(),
                                 state->wheel2()->rot()->e2(), state->wheel2()->rot()->e3()})}),
            SynMessage::Pose(
                {ChVector<>({state->wheel3()->pos()->x(), state->wheel3()->pos()->y(), state->wheel3()->pos()->z()}),
                 ChQuaternion<>({state->wheel3()->rot()->e0(), state->wheel3()->rot()->e1(),
                                 state->wheel3()->rot()->e2(), state->wheel3()->rot()->e3()})}));
    }

    virtual FlatBufferMessage GenerateMessage(flatbuffers::FlatBufferBuilder& builder) {
        std::shared_ptr<State> state = std::static_pointer_cast<State>(m_state);
        flatbuffers::Offset<SynFlatBuffers::Agent::Pose> chassis = CreatePose(builder, state->chassis);
        flatbuffers::Offset<SynFlatBuffers::Agent::Pose> wheel0 = CreatePose(builder, state->wheel0);
        flatbuffers::Offset<SynFlatBuffers::Agent::Pose> wheel1 = CreatePose(builder, state->wheel1);
        flatbuffers::Offset<SynFlatBuffers::Agent::Pose> wheel2 = CreatePose(builder, state->wheel2);
        flatbuffers::Offset<SynFlatBuffers::Agent::Pose> wheel3 = CreatePose(builder, state->wheel3);

        flatbuffers::Offset<SynFlatBuffers::Agent::State> flatbuffer_state = SynFlatBuffers::Agent::CreateState(
            builder, SynFlatBuffers::Agent::Type_Sedan_State,
            SynFlatBuffers::Agent::Sedan::CreateState(builder, state->time, chassis, wheel0, wheel1, wheel2, wheel3)
                .Union());

        FlatBufferMessage message = flatbuffers::Offset<SynFlatBuffers::Message>(
            SynFlatBuffers::CreateMessage(builder, SynFlatBuffers::Type_Agent_State, flatbuffer_state.Union(),
                                          builder.CreateString(std::to_string(m_id))));
        return message;
    }

  private:
    flatbuffers::Offset<SynFlatBuffers::Agent::Pose> CreatePose(flatbuffers::FlatBufferBuilder& builder,
                                                                SynMessage::Pose& syn_pose) {
        flatbuffers::Offset<SynFlatBuffers::Agent::Pose> fb_pose = SynFlatBuffers::Agent::CreatePose(
            builder, SynFlatBuffers::Agent::CreateVector(builder, syn_pose.pos.x(), syn_pose.pos.y(), syn_pose.pos.z()),
            SynFlatBuffers::Agent::CreateQuaternion(builder, syn_pose.orientation.e0(), syn_pose.orientation.e1(),
                                                    syn_pose.orientation.e2(), syn_pose.orientation.e3()));
        return fb_pose;
    }
};
