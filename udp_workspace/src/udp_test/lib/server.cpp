#include "udp_test/server.h"

void log(const char* msg) { printf("[SERVER] :: %s",msg); }
void log(const char* msg1, const char* msg2) { printf("[SERVER] :: %s %s", msg1, msg2); }

Server::Server() {
  if ((m_server_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    log("Failed to create socket");
    return;
  }

  memset(&m_ser_addr, 0, sizeof(m_ser_addr));
  m_ser_addr.sin_family = AF_INET;
  m_ser_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
  m_ser_addr.sin_port = htons(SERVER_PORT);

  m_ret = bind(m_server_fd, (struct sockaddr *)&m_ser_addr, sizeof(m_ser_addr));
  if (m_ret < 0) {
    log("Failed to bind socket");
    return;
  }
}

void Server::HandleMsgs() {
  memset(m_buffer, 0, BUFF_LEN);
  m_len = sizeof(m_client_addr);
  m_count = recvfrom(m_server_fd, m_buffer, BUFF_LEN, 0,
                     (struct sockaddr *)&m_client_addr, &m_len);
  if (m_count == -1) {
    log("Failed to recieve data");
    return;
  }

  log("Data recieved from client : ", m_buffer);
  memset(m_buffer, 0, BUFF_LEN);
  sprintf(m_buffer, "I have recieved %d bytes of data!", m_count);
  log("Data sent to client : ", m_buffer);
  sendto(m_server_fd, m_buffer, BUFF_LEN, 0, (struct sockaddr *)&m_client_addr,
         m_len);
}
