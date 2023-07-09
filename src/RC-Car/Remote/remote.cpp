#include "CYdLidar.h"
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <string>
#include <sstream>
#include <signal.h>

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_sdk.lib")
#endif

// Client side implementation of UDP client-server model
   
#define PORT     8080
#define MAXLINE 1024
   
// Define a struct to hold the sensor readings
struct SensorData {
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float magX, magY, magZ;
    long long timestamp;
};

// Define a struct to hold the lidar readings
struct LIDARData {
    float angle;
    float range;
    long long timestamp;
};

// Function to convert a string back into a SensorData object
SensorData stringToSensorData(const std::string& dataStr) {
    std::stringstream ss(dataStr);
    SensorData data;
    char comma; // for reading the commas in the string

    ss >> data.accelX >> comma
       >> data.accelY >> comma
       >> data.accelZ >> comma
       >> data.gyroX >> comma
       >> data.gyroY >> comma
       >> data.gyroZ >> comma
       >> data.magX >> comma
       >> data.magY >> comma
       >> data.magZ >> comma
       >> data.timestamp;

    return data;
}

// Function to convert the sensor data to a string for sending over the socket
LIDARData stringToLIDARData(const std::string& dataStr) {
    std::stringstream ss(dataStr);
    LIDARData data;
    char comma; // for reading the commas in the string

    ss >> data.angle >> comma
       >> data.range >> comma
       >> data.timestamp;

    return data;
}

void saveCursorPosition() {
  printf("\033[s");
}

void restoreCursorPosition() {
  printf("\033[u");
}

void lineUP(short int times) {
  printf("\033[%iA", times);
}

void lineDOWN(short int times) {
  printf("\033[%iB", times);
}

void clearLine() {
    printf("\033[K");
}

void exitHandler(int s){
    restoreCursorPosition();
    printf("Caught signal %d\n",s);
    exit(1); 

}

// Driver code
int main() {

    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = exitHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

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

    std::cout << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl << std::endl;

    saveCursorPosition();

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

        std::string firstWord = dataStr.substr(0, dataStr.find(","));


        

        if(firstWord == "LIDAR") {
            // Print received data
            dataStr.erase(0, 6);
            LIDARData data = stringToLIDARData(dataStr);
            restoreCursorPosition();
            lineUP(9);
            std::cout << "LIDAR:";
            restoreCursorPosition();
            lineUP(8);
            std::cout << "\tAngle: " << data.angle << std::endl; 
            restoreCursorPosition();
            lineUP(7);
            std::cout << "\tRange: " << data.range << std::endl;
            restoreCursorPosition();
            lineUP(6);
            std::cout << "\tTimestamp: " << data.timestamp << std::endl;
        } else if(firstWord == "SENSOR") {
            dataStr.erase(0, 7);
            SensorData data = stringToSensorData(dataStr);
            restoreCursorPosition();
            lineUP(5);
            clearLine();
            std::cout << "Sensor:" << std::endl;
            restoreCursorPosition();
            lineUP(4);
            clearLine();
            std::cout << "\tAccel(x,y,z): " << data.accelX << "," << data.accelY << "," << data.accelZ << std::endl;
            restoreCursorPosition();
            lineUP(3);
            clearLine();
            std::cout << "\tGyro(x,y,z): " << data.gyroX << "," << data.gyroY << "," << data.gyroZ << std::endl;
            restoreCursorPosition();
            lineUP(2);
            clearLine();
            std::cout << "\tMag(x,y,z): " << data.magX << "," << data.magY << "," << data.magZ << std::endl;
            restoreCursorPosition();
            lineUP(1);
            clearLine();
            std::cout << "\tTimestamp: " << data.timestamp << std::endl;
        }

        
    }

    // Close the socket
    close(sockfd);

    return 0;
}