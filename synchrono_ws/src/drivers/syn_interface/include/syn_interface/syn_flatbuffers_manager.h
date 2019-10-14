#pragma once

#include "syn_interface/syn_interface_message_generated.h"

typedef std::vector<flatbuffers::Offset<SynInterfaceMessage::Message>>
    MessageVector;
typedef const SynInterfaceMessage::Messages *Msgs;

class SynFlatBuffersManager {
private:
  flatbuffers::FlatBufferBuilder builder_;

  MessageVector message_vector_;

  std::vector<uint8_t> buffer_;

public:
  SynFlatBuffersManager();
  ~SynFlatBuffersManager();

  void
  AddMessage(std::shared_ptr<flatbuffers::Offset<SynInterfaceMessage::Message>>
                 message);

  void Finish();

  int32_t GetSize();

  uint8_t *GetBufferPointer();

  bool HasMessage(std::string name);

  const SynInterfaceMessage::Message *GetMessage(std::string name);

  void ReadMessage(std::vector<uint8_t> buffer);

  void Reset();

  flatbuffers::FlatBufferBuilder &GetBuilder() { return builder_; }

  std::vector<uint8_t> &Buffer() { return buffer_; }
};
