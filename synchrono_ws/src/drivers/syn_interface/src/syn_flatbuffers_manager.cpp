#include "syn_interface/syn_flatbuffers_manager.h"

#include <iostream>

SynFlatBuffersManager::SynFlatBuffersManager() {}

SynFlatBuffersManager::~SynFlatBuffersManager() {}

void SynFlatBuffersManager::AddMessage(
    std::shared_ptr<flatbuffers::Offset<SynInterfaceMessage::Message>>
        message) {
  message_vector_.push_back(*message);
}

void SynFlatBuffersManager::Finish() {
  flatbuffers::Offset<SynInterfaceMessage::Messages> messages =
      SynInterfaceMessage::CreateMessages(
          builder_, builder_.CreateVector(message_vector_));

  builder_.FinishSizePrefixed(messages);
}

int32_t SynFlatBuffersManager::GetSize() { return builder_.GetSize(); }

uint8_t *SynFlatBuffersManager::GetBufferPointer() {
  return builder_.GetBufferPointer();
}

bool SynFlatBuffersManager::HasMessage(std::string name) {
  const SynInterfaceMessage::Messages *msgs =
      flatbuffers::GetRoot<SynInterfaceMessage::Messages>(buffer_.data());

  for (auto message : (*msgs->messages())) {
    if (message->id()->str().compare(name) == 0) {
      return true;
    }
  }
  return false;
}

const SynInterfaceMessage::Message *
SynFlatBuffersManager::GetMessage(std::string name) {
  const SynInterfaceMessage::Messages *msgs =
      flatbuffers::GetRoot<SynInterfaceMessage::Messages>(buffer_.data());

  for (auto message : (*msgs->messages())) {
    if (message->id()->str().compare(name) == 0) {
      return message;
    }
  }
}

void SynFlatBuffersManager::Reset() {
  builder_.Clear();
  message_vector_.clear();
}
