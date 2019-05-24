#include "udp_test/client.h"

void log(const char* msg) { printf("[CLIENT] :: %s",msg); }
void log(const char* msg1, const char* msg2) { printf("[CLIENT] :: %s %s", msg1, msg2); }

Client::Client() {
  if((m_client_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    log("Failed to create socket");
    return;
  }

  memset(&m_ser_addr, 0, sizeof(m_ser_addr));
  m_ser_addr.sin_family = AF_INET;
  m_ser_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
  m_ser_addr.sin_port = htons(SERVER_PORT);
}

void Client::HandleMsgs() {
  m_len = sizeof(*(struct sockaddr *)&m_ser_addr);
  log("Message from client: ", m_buffer);

  sendto(m_client_fd, m_buffer, BUFF_LEN, 0, (struct sockaddr *)&m_ser_addr, m_len);
  memset(m_buffer, 0, BUFF_LEN);
  recvfrom(m_client_fd, m_buffer, BUFF_LEN, 0, (struct sockaddr *)&m_src, &m_len);
  log("Message from server: ", m_buffer);
  sleep(1);
}
