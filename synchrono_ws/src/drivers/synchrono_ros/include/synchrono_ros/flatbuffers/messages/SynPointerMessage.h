#pragma once

#include "synchrono_ros/flatbuffers/messages/SynMessage.h"

class  SynPointerMessage : public SynMessage {
  public:
    SynPointerMessage(int id, uint8_t* buff, int bytes)
        : SynMessage(id, SynMessage::VEH), m_buff(buff), m_bytes(bytes) {}

    virtual void GenerateState(const SynFlatBuffers::Message* message) {}

    virtual FlatBufferMessage GenerateMessage(flatbuffers::FlatBufferBuilder& builder) {
        flatbuffers::Offset<SynFlatBuffers::Pointer> pointer =
            SynFlatBuffers::CreatePointer(builder, builder.CreateVector<unsigned char>(m_buff, m_bytes));

        FlatBufferMessage message = flatbuffers::Offset<SynFlatBuffers::Message>(
            SynFlatBuffers::CreateMessage(builder, SynFlatBuffers::Type_Pointer, pointer.Union()));

        return message;
    }

  private:
    uint8_t* m_buff;
    int m_bytes;
};
