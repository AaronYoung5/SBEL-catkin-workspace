#pragma once

#include <ros/ros.h>

#include "synchrono_ros/flatbuffers/SynFlatBuffers_generated.h"
#include "synchrono_ros/flatbuffers/messages/SynMessage.h"

class SynComponent {
public:
  enum Type { SENDER, RECEIVER, SENDERANDRECEIVER, UPDATER };

  SynComponent(Type type, std::string id, float update_rate)
      : m_type(type), m_id(id), m_update_rate(update_rate) {}

  SynComponent(Type type, std::string id) : m_type(type), m_id(id) {}

  SynComponent(Type type) : m_type(type) {}

  virtual void Advance(double time) = 0;

  Type GetType() { return m_type; }

  std::string GetID() { return m_id; }
  void SetID(std::string id) { m_id = id; }

  float GetUpdateRate() { return m_update_rate; }
  void SetUpdateRate(float update_rate) { m_update_rate = update_rate; }

  unsigned int GetNumUpdates() { return m_num_updates; }
  void IncrementNumUpdates() { m_num_updates++; }

  bool ShouldUpdate(double ch_time) {
    return m_has_advanced;
  }

  bool IsConditional() { return m_is_conditional; }
  void MakeConditional() { m_is_conditional = true; }

  std::shared_ptr<SynMessage> GetMessage() { return m_msg; }

protected:
  std::string m_id;

  float m_update_rate = 0;

  unsigned int m_num_updates = 0;

  Type m_type;

  bool m_is_conditional = false;

  bool m_has_advanced = false;

  std::shared_ptr<SynMessage> m_msg;
};
