#include <arpa/inet.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#define SERVER_PORT 8080
#define BUFF_LEN 512

class Client {
private:
  // Private class members
  int m_client_fd;
  struct sockaddr_in m_ser_addr;

  char m_buffer[BUFF_LEN] = "Test msg from client";
  socklen_t m_len;
  struct sockaddr_in m_src;

public:
  // Public methods
  Client();
  void HandleMsgs();
  void Close() { close(m_client_fd); }
};
