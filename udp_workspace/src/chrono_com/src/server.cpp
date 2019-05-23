#include <arpa/inet.h>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#define SERVER_PORT 8888
#define BUFF_LEN 1024

void log(std::string msg) { std::cout << "[SERVER] :: " << msg << std::endl; }
void log(std::string msg1, std::string msg2) {
  std::cout << "[SERVER] :: " << msg1 << msg2 << std::endl;
}

void quit(std::string msg) {
  log(msg);
  return -1;
}

class Server {
private:
  // Private class variables
  // For server initialization
  int m_server_fd, m_ret;
  struct sockaddr_in m_ser_addr;

  // For msg handling
  char m_buffer[BUFF_LEN];
  socklen_t m_len;
  int m_count;
  struct sockaddr_in m_client_addr;

public:
  // Public methods
  Server();

  void HandleMsg();
};

Server::Server() {
  if ((m_server_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    quit("Failed to create socket");
  }

  memset(&m_ser_addr, 0, sizeof(m_ser_addr));
  m_ser_addr.sin_family = AF_INET;
  m_ser_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
  m_ser_addr.sin_port = htons(SERVER_PORT);

  m_ret = bind(m_server_fd, (struct sockaddr *)&m_ser_addr, sizeof(m_ser_addr));
  if (m_ret < 0) {
    quit("Failed to bind socket");
  }
}

void Server::HandleMsg() {
  memset(m_buf, 0, BUFF_LEN);
  m_len = sizeof(m_client_addr);
  count = recvfrom(m_server_fd, m_buf, BUFF_LEN, 0,
                   (struct sockaddr *)&client_addr, &len);
  if (count == -1) {
    quit("Failed to recieve data");
  }

  log("Data recieved from client :", m_buf);
  memset(m_buf);
}

void handle_udp_msg(int fd) {
  char buf[BUFF_LEN];
  socklen_t len;
  int count;
  struct sockaddr_in clent_addr;
  while (1) {
    memset(buf, 0, BUFF_LEN);
    len = sizeof(clent_addr);
    count =
        recvfrom(fd, buf, BUFF_LEN, 0, (struct sockaddr *)&clent_addr, &len);
    if (count == -1) {
      printf("recieve data fail!\n");
      return;
    }
    printf("client:%s\n", buf);
    memset(buf, 0, BUFF_LEN);
    sprintf(buf, "I have recieved %d bytes data!\n", count);
    printf("server:%s\n", buf);
    sendto(fd, buf, BUFF_LEN, 0, (struct sockaddr *)&clent_addr, len);
  }
}

int main(int argc, char *argv[]) { close(server_fd); }
