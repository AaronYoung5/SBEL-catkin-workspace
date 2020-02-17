#pragma once

#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector.h"

#include "synchrono_ros/flatbuffers/SynFlatBuffers_generated.h"

using namespace chrono;

typedef flatbuffers::Offset<SynFlatBuffers::Message> FlatBufferMessage;
// typedef const SynFlatBuffers::Message* Message;
class SynMessage {
public:
  struct Pose {
    ChVector<> pos;
    ChQuaternion<> orientation;
  };

  struct State {
    double time;

    State(double time) : time(time) {}
    State() : time(0.0) {}
  };

  enum Type { MAP, SPAT, VEH, TRAFFIC_LIGHT, CAMERA, CONTROL };

  SynMessage(int id, Type type)
      : m_id(id), m_type(type),
        m_state(std::make_shared<SynMessage::State>(0.0)) {}

  virtual FlatBufferMessage
  GenerateMessage(flatbuffers::FlatBufferBuilder &builder) = 0;
  virtual void GenerateState(const SynFlatBuffers::Message *message) = 0;

  Type GetType() { return m_type; }
  unsigned int GetID() { return m_id; }
  double GetTime() { return m_time; }
  std::shared_ptr<State> &GetState() { return m_state; }

protected:
  int m_id;
  Type m_type;
  double m_time;
  std::shared_ptr<State> m_state;
};
