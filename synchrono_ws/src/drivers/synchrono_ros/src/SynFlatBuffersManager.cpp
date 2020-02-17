#include "synchrono_ros/flatbuffers/SynFlatBuffersManager.h"

// Constructor
SynFlatBuffersManager::SynFlatBuffersManager() {}

// Destructor
SynFlatBuffersManager::~SynFlatBuffersManager() {}

// When a message is received through MPI, the byte array (buffer) is stored in
// the class variable m_buffer. This buffer is then converted to a flatbuffer
// message. The ith message is then retrieved from this buffer message.
const SynFlatBuffers::Message *SynFlatBuffersManager::Get(int i) {
  const SynFlatBuffers::Buffer *msgs =
      flatbuffers::GetRoot<SynFlatBuffers::Buffer>(m_buffer.data());
  const SynFlatBuffers::Message *message = msgs->buffer()->Get(i - 1);
  return message;
}

// When a message is received through TCP (interface), the byte array (buffer)
// is stored in the class variable m_buffer. This buffer is then converted to a
// flatbuffer message. The message with the same id is then retrieved from this
// buffer message.
const SynFlatBuffers::Message *SynFlatBuffersManager::Get(std::string id) {
  const SynFlatBuffers::Buffer *msgs =
      flatbuffers::GetRoot<SynFlatBuffers::Buffer>(m_buffer.data());
  for (auto message : (*msgs->buffer())) {
    std::cout << "TESTSETSETSETSETST" << std::endl;
    if (message->id()->str().compare(id) == 0)
      return message;
  }
  return nullptr;
}

// Adds a SynMessage to the flatbuffer message buffer.
void SynFlatBuffersManager::AddMessage(SynMessage *message) {
  m_message_vector.push_back(message->GenerateMessage(m_builder));
}

// Adds a SynFlatBuffers Message to the flatbuffer message buffer.
void SynFlatBuffersManager::AddMessage(
    flatbuffers::Offset<SynFlatBuffers::Message> message) {
  m_message_vector.push_back(message);
}

// Completes the flatbuffer message. Creates a buffer message, of which stores
// every message in an vector.
void SynFlatBuffersManager::Finish() {
  flatbuffers::Offset<SynFlatBuffers::Buffer> buffer =
      SynFlatBuffers::CreateBuffer(m_builder,
                                   m_builder.CreateVector(m_message_vector));

  m_builder.Finish(buffer);
}

// Completes the flatbuffer message with a 4 bytes in the front of the buffer
// which has the size of the byte array. Creates a buffer message, of which
// stores every message in an vector.
void SynFlatBuffersManager::FinishSizePrefixed() {
  flatbuffers::Offset<SynFlatBuffers::Buffer> buffer =
      SynFlatBuffers::CreateBuffer(m_builder,
                                   m_builder.CreateVector(m_message_vector));

  m_builder.FinishSizePrefixed(buffer);
}

// Gets the size of the buffer message.
int32_t SynFlatBuffersManager::GetSize() { return m_builder.GetSize(); }

// Gets the buffer pointer.
uint8_t *SynFlatBuffersManager::GetBufferPointer() {
  return m_builder.GetBufferPointer();
}

// Reset the flatbuffer. Must be called, otherwise messages will just continue
// to be added to the vector (memory leak).
void SynFlatBuffersManager::Reset() {
  m_builder.Clear();
  m_message_vector.clear();
}
