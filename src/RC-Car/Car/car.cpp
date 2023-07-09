#include "CYdLidar.h"
#include <thread>
#include <mutex>
#include <string>
#include <iostream>
#include <sys/socket.h> // Required for socket programming
#include <netinet/in.h> // Required for internet domain addresses
#include <arpa/inet.h> // Required for inet_addr function
#include <unistd.h> // Required for close function
#include <chrono> // Required for timestamps

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_sdk.lib")
#endif

// Server side implementation of UDP client-server model
   
#define PORT     8080
#define MAXLINE 1024

// Function to initialize the lidar with the setting for the x4
void InitializeLidar(CYdLidar &PtrLaser)
{
  //////////////////////string property/////////////////
  /// Lidar ports
  std::map<std::string, std::string> ports = ydlidar::lidarPortList();
  std::string port = "/dev/ydlidar";
  if (!ports.empty())
  {
    port = ports.begin()->second;
  }
  /// lidar port
  PtrLaser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
  /// ignore array
  std::string ignore_array;
  ignore_array.clear();
  PtrLaser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(),
                       ignore_array.size());

  //////////////////////int property/////////////////
  /// lidar baudrate
  int optval = 128000;
  PtrLaser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));
  /// tof lidar
  optval = TYPE_TRIANGLE;
  PtrLaser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
  /// device type
  optval = YDLIDAR_TYPE_SERIAL;
  PtrLaser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
  /// sample rate
  optval = 5;
  PtrLaser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
  /// abnormal count
  optval = 4;
  PtrLaser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));

  //////////////////////bool property/////////////////
  /// fixed angle resolution
  bool b_optvalue = false;
  PtrLaser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
  /// rotate 180
  PtrLaser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
  /// Counterclockwise
  PtrLaser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
  b_optvalue = true;
  PtrLaser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
  /// one-way communication
  b_optvalue = false;
  PtrLaser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
  /// intensity
  b_optvalue = false;
  PtrLaser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
  /// Motor DTR
  b_optvalue = true;
  PtrLaser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));
  /// HeartBeat
  b_optvalue = false;
  PtrLaser.setlidaropt(LidarPropSupportHeartBeat, &b_optvalue, sizeof(bool));

  //////////////////////float property/////////////////
  /// unit: °
  float f_optvalue = 180.0f;
  PtrLaser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
  f_optvalue = -180.0f;
  PtrLaser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
  /// unit: m
  f_optvalue = 10.f;
  PtrLaser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
  f_optvalue = 0.1f;
  PtrLaser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
  /// unit: Hz
  f_optvalue = 12.f;
  PtrLaser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));
}

// Driver code
int main() {
    std::cout <<  "\033[1;35m"
                "  _____           \n"
                " / ____|          \n"
                "| |     __ _ _ __ \n"
                "| |    / _` | '__|\n"
                "| |___| (_| | |   \n"
                " \\_____\\__,_|_|   \n"
                "\033[0m" << std::endl;

    // init system signal
    ydlidar::os_init();

    CYdLidar laser;

    InitializeLidar(laser);

    // initialize SDK and LiDAR
    bool ret = laser.initialize();
    if (ret)
    { // success
        // Start the device scanning routine which runs on a separate thread and enable motor.
        ret = laser.turnOn();
    }
    else
    {
        fprintf(stderr, "%s\n", laser.DescribeError());
        fflush(stderr);
    }

    // Create a socket
    // AF_INET is the address family for IPv4
    // SOCK_DGRAM is the socket type for UDP
    // 0 is the protocol value for IP
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("Failed to create socket");
        return 1;
    }

    // Specify the address and port of the remote client
    struct sockaddr_in servaddr;
    servaddr.sin_family = AF_INET; // Address family for IPv4
    servaddr.sin_port = htons(PORT); // Port number, converted to network byte order
    servaddr.sin_addr.s_addr = inet_addr("127.0.0.1"); // IP address, converted to network byte order

        
    while (ret && ydlidar::os_isOk())
    {
        LaserScan scan;
        if (laser.doProcessSimple(scan))
        {
            for(int i = 0; i < scan.points.size(); i++) {
                // Get the current time
                auto now = std::chrono::system_clock::now();
                auto duration = now.time_since_epoch();
                auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

                // Prepare the data to be sent
                std::string data = "Angle: " + std::to_string(scan.points[i].angle) + " Range: " + std::to_string(scan.points[i].range) + " Timestamp: " + std::to_string(milliseconds) + "\n";
                // Send the data over the socket to the specified address and port
                sendto(sockfd, data.c_str(), data.size(), 0, (struct sockaddr *)&servaddr, sizeof(servaddr));
            }

            // Get the current time
            auto now = std::chrono::system_clock::now();
            auto duration = now.time_since_epoch();
            auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

            std::cout << "Points Sent: " + std::to_string(scan.points.size()) + " Timestamp: " + std::to_string(milliseconds) + "\n";
        }
        else
        {
            fprintf(stderr, "Failed to get Lidar Data\n");
            fflush(stderr);
        }
    }

    // Close the socket
    close(sockfd);

    // Stop the device scanning thread and disable motor.
    laser.turnOff();
    // Uninitialize the SDK and Disconnect the LiDAR.
    laser.disconnecting();

    return 0;
}