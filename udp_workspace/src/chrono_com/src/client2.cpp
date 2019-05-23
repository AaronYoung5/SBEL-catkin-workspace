#include <arpa/inet.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#define SERVER_PORT 8888
#define BUFF_LEN 512

void udp_msg_sender(int fd, struct sockaddr *dst) {

  socklen_t len;
  struct sockaddr_in src;
  while (1) {
    char buf[BUFF_LEN] = "TEST UDP MSG!\n";
    len = sizeof(*dst);
    printf("client:%s\n", buf);
    sendto(fd, buf, BUFF_LEN, 0, dst, len);
    memset(buf, 0, BUFF_LEN);
    recvfrom(fd, buf, BUFF_LEN, 0, (struct sockaddr *)&src, &len);
    printf("server:%s\n", buf);
    sleep(1);
  }
}

int main(int argc, char *argv[]) {
  int client_fd;
  struct sockaddr_in ser_addr;

  client_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (client_fd < 0) {
    printf("create socket fail!\n");
    return -1;
  }

  memset(&ser_addr, 0, sizeof(ser_addr));
  ser_addr.sin_family = AF_INET;
  ser_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
  ser_addr.sin_port = htons(SERVER_PORT);

  udp_msg_sender(client_fd, (struct sockaddr *)&ser_addr);

  close(client_fd);

  return 0;
}
