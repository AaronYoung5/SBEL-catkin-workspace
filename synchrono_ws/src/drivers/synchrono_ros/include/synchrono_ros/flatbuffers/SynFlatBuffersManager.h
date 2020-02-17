#pragma once

#include <vector>

#include "synchrono_ros/flatbuffers/messages/SynMessage.h"
#include "synchrono_ros/flatbuffers/SynFlatBuffers_generated.h"

class SynFlatBuffersManager {
  public:
    SynFlatBuffersManager();
    ~SynFlatBuffersManager();

    void AddMessage(SynMessage* message);
    void AddMessage(flatbuffers::Offset<SynFlatBuffers::Message> message);

    void Finish();
    void FinishSizePrefixed();

    void Reset();

    const SynFlatBuffers::Message* Get(int i);
    const SynFlatBuffers::Message* Get(std::string id);

    // Getters
    int32_t GetSize();
    uint8_t* GetBufferPointer();
    std::vector<uint8_t>& GetBuffer() { return m_buffer; }
    flatbuffers::FlatBufferBuilder& GetBuilder() { return m_builder; }

  private:
    flatbuffers::FlatBufferBuilder m_builder;

    std::vector<flatbuffers::Offset<SynFlatBuffers::Message>> m_message_vector;

    // When a message is received, the byte array is stored in this buffer
    std::vector<uint8_t> m_buffer;
};
