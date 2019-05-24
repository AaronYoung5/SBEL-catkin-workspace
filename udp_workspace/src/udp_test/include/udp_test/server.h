#include <arpa/inet.h>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#define SERVER_PORT 8080
#define BUFF_LEN 512

class Server {
private:
  // Private class variables
  int m_server_fd, m_ret;
  struct sockaddr_in m_ser_addr;

  char m_buffer[BUFF_LEN];
  socklen_t m_len;
  int m_count;
  struct sockaddr_in m_client_addr;

public:
  // Public methods
  Server();
  void HandleMsgs();
  void Close() { close(m_server_fd); }
};
