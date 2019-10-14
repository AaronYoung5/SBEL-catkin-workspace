#pragma once

#include <ros/ros.h>

#include "syn_interface/syn_interface_message_generated.h"

class SynInterfaceComponent {
public:
  enum SynComponentType { Sender, Receiver, SenderAndReceiver };

protected:
  std::string name_;

  SynComponentType type_;

public:
  SynInterfaceComponent(SynComponentType type, std::string name)
      : type_(type), name_(name) {}

  virtual std::shared_ptr<flatbuffers::Offset<SynInterfaceMessage::Message>> toFlatBuffer(flatbuffers::FlatBufferBuilder &builder_) = 0;

  virtual void fromFlatBuffer(const SynInterfaceMessage::Message *message) = 0;

  SynComponentType GetType() { return type_; }

  std::string GetName() { return name_; }
  void SetName(std::string name) { name_ = name; }
};
