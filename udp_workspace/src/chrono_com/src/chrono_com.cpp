// Includes for ROS
#include "ros/ros.h"

// Includes for server side implementatino of UDP client-servel model
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#define PORT 8080
#define MAXLINE 1024

int sockfd;
char buffer[MAXLINE];
const char *hello = "Hello from server";
struct sockaddr_in servaddr, cliaddr;

void initUDP() {
  // Creating socket file descriptor
  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    perror("socket craetion failed");
    exit(EXIT_FAILURE);
  }

  memset(&servaddr, 0, sizeof(servaddr));
  memset(&cliaddr, 0, sizeof(cliaddr));

  // Filling server information
  servaddr.sin_family = AF_INET; // IPv4
  servaddr.sin_addr.s_addr = INADDR_ANY;
  servaddr.sin_port = htons(PORT);

  // Bind the socket with the server address
  if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
    perror("bind failed");
    exit(EXIT_FAILURE);
  }
}

void sendUDP() {
  uint len, n;
  n = recvfrom(sockfd, (char *)buffer, MAXLINE, MSG_WAITALL,
               (struct sockaddr *)&cliaddr, &len);
  std::cout << "Client : " << buffer << std::endl;
  sendto(sockfd, (const char *)hello, strlen(hello), MSG_CONFIRM,
         (const struct sockaddr *)&cliaddr, len);
  std::cout << "Hello message sent" << std::endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "chrono_com");

  std::cout << "1" << std::endl;

  initUDP();

  std::cout << "2" << std::endl;

  sendUDP();

  std::cout << "3" << std::endl;

  ros::shutdown();
}
