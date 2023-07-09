#include "CYdLidar.h"
#include <bits/stdc++.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_sdk.lib")
#endif

// Client side implementation of UDP client-server model
   
#define PORT     8080
#define MAXLINE 1024
   
// Driver code
int main() {
    std::cout <<  "\033[1;35m"
                " _____                      _       \n"
                "|  __ \\                    | |      \n"
                "| |__) |___ _ __ ___   ___ | |_ ___ \n"
                "|  _  // _ \\ '_ ` _ \\ / _ \\| __/ _ \\\n"
                "| | \\ \\  __/ | | | | | (_) | ||  __/\n"
                "|_|  \\_\\___|_| |_| |_|\\___/ \\__\\___|\n"
                "\033[0m" << std::endl;
    int sockfd;
    char buffer[MAXLINE];
    char *hello;
    struct sockaddr_in     servaddr;
   
    // Creating socket file descriptor
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }
   
    memset(&servaddr, 0, sizeof(servaddr));
       
    // Filling server information
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(PORT);
    servaddr.sin_addr.s_addr = INADDR_ANY;
       
    int n;
    socklen_t len;
       
    while(true) {
        std::cout << "Message To Send: ";
        std::cin >> hello;
        std::cout << "Message: " << hello << std::endl;
        
        sendto(sockfd, (char *)hello, strlen(hello),
            MSG_CONFIRM, (const struct sockaddr *) &servaddr, 
                sizeof(servaddr));
        std::cout<<"Hello message sent."<<std::endl;
            
        n = recvfrom(sockfd, (char *)buffer, MAXLINE, 
                    MSG_WAITALL, (struct sockaddr *) &servaddr,
                    &len);
        buffer[n] = '\0';
        std::cout<<"Server :"<<buffer<<std::endl;
    
        close(sockfd);
    }

    return 0;
}