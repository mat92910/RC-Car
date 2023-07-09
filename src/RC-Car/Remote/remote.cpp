#include "CYdLidar.h"
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <string>
#include <sstream>

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_sdk.lib")
#endif

// Client side implementation of UDP client-server model
   
#define PORT     8080
#define MAXLINE 1024
   
// Define the SensorData struct (same as before)
struct SensorData {
    float angle;
    float range;
    long long timestamp;
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float magX, magY, magZ;
};

// Function to convert a string back into a SensorData object
SensorData stringToSensorData(const std::string& dataStr) {
    std::stringstream ss(dataStr);
    SensorData data;
    char comma; // for reading the commas in the string

    ss >> data.angle >> comma
       >> data.range >> comma
       >> data.timestamp >> comma
       >> data.accelX >> comma
       >> data.accelY >> comma
       >> data.accelZ >> comma
       >> data.gyroX >> comma
       >> data.gyroY >> comma
       >> data.gyroZ >> comma
       >> data.magX >> comma
       >> data.magY >> comma
       >> data.magZ;

    return data;
}

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

        std::string dataStr = buffer;
        SensorData data = stringToSensorData(dataStr);

        // Print received data
        std::cout << "Angle: " << data.angle << std::endl;
    }

    // Close the socket
    close(sockfd);

    return 0;
}