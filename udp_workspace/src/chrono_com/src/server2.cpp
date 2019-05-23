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

int main(int argc, char *argv[]) {
  int server_fd, ret;
  struct sockaddr_in ser_addr;

  server_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (server_fd < 0) {
    printf("create socket fail!\n");
    return -1;
  }

  memset(&ser_addr, 0, sizeof(ser_addr));
  ser_addr.sin_family = AF_INET;
  ser_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
  ser_addr.sin_port = htons(SERVER_PORT);

  ret = bind(server_fd, (struct sockaddr *)&ser_addr, sizeof(ser_addr));
  if (ret < 0) {
    printf("socket bind fail!\n");
    return -1;
  }

  handle_udp_msg(server_fd);

  close(server_fd);
}
