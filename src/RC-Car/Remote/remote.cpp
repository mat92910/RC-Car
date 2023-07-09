#include "CYdLidar.h"
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>

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
    // Create a socket
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("Failed to create socket");
        return 1;
    }

    // Specify the address and port to bind to
    struct sockaddr_in servaddr;
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(PORT); // replace with your port
    servaddr.sin_addr.s_addr = INADDR_ANY; // bind to any local address

    // Bind the socket to the specified address and port
    if (bind(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        perror("Failed to bind socket");
        return 1;
    }

    // Buffer to store incoming data
    char buffer[1024];

    // Client loop
    while (true) {
        // Clear the buffer
        memset(buffer, 0, sizeof(buffer));

        // Receive data
        struct sockaddr_in cliaddr;
        socklen_t len = sizeof(cliaddr);
        int n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr *)&cliaddr, &len);
        if (n < 0) {
            perror("Failed to receive data");
            break;
        }

        // Print received data
        std::cout << buffer;
    }

    // Close the socket
    close(sockfd);

    return 0;
}