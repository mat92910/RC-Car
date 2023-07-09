#include "CYdLidar.h"
#include <thread>
#include <mutex>
#include <string>
#include <sstream>
#include <iostream>
#include <sys/socket.h> // Required for socket programming
#include <netinet/in.h> // Required for internet domain addresses
#include <arpa/inet.h> // Required for inet_addr function
#include <unistd.h> // Required for close function
#include <chrono> // Required for timestamps
#include <wiringPiI2C.h>

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_sdk.lib")
#endif

// Server side implementation of UDP client-server model
   
#define PORT     8080
#define MAXLINE 1024

// IMU
#define MPU9250_ADDR 0x68
#define AK8963_ADDR  0x0C

#define USER_CTRL 0x6A
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C

#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F

#define GYRO_XOUT_H  0x43
#define GYRO_YOUT_H  0x45
#define GYRO_ZOUT_H  0x47

#define INT_PIN_CFG  0x37

#define AK8963_ST1 0x02
#define AK8963_XOUT_L 0x03
#define AK8963_CNTL1 0x0A
#define AK8963_CNTL2 0x0B

#define I2C_SLV0_ADDR 0x25
#define I2C_SLV0_REG 0x26
#define I2C_SLV0_CTRL 0x27
#define I2C_SLV0_DO 0x63

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

// Function to convert the sensor data to a string for sending over the socket
std::string sensorDataToString(const SensorData& data) {
    std::stringstream ss;
    ss << "SENSOR" << ","
       << data.accelX << ","
       << data.accelY << ","
       << data.accelZ << ","
       << data.gyroX << ","
       << data.gyroY << ","
       << data.gyroZ << ","
       << data.magX << ","
       << data.magY << ","
       << data.magZ << ","
       << data.timestamp;
    return ss.str();
}

// Function to convert the sensor data to a string for sending over the socket
std::string LIDARDataToString(const LIDARData& data) {
    std::stringstream ss;
    ss << "LIDAR" << ","
       << data.angle << ","
       << data.range << ","
       << data.timestamp;
    return ss.str();
}

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

    // Initialize the I2C interface for the MPU9250
    int fd_mpu9250 = wiringPiI2CSetup(MPU9250_ADDR);
    if (fd_mpu9250 == -1) {
        std::cerr << "Failed to initialize I2C for MPU9250\n";
        return 1;
    }

    // Reset the MPU9250
    wiringPiI2CWriteReg8(fd_mpu9250, PWR_MGMT_1, 0x80);
    usleep(100000); // Wait for 100 ms after reset

    // Wake up the MPU9250
    wiringPiI2CWriteReg8(fd_mpu9250, PWR_MGMT_1, 0x00);
    usleep(100000); // Wait for 100 ms after wake up

    // Enable the I2C master mode
    wiringPiI2CWriteReg8(fd_mpu9250, USER_CTRL, 0x20);

    // Set the I2C slave address of the AK8963 and enable reading
    wiringPiI2CWriteReg8(fd_mpu9250, I2C_SLV0_ADDR, AK8963_ADDR | 0x80);

    // Set the register to start reading
    wiringPiI2CWriteReg8(fd_mpu9250, I2C_SLV0_REG, AK8963_XOUT_L);

    // Enable I2C and set the number of bytes to read
    wiringPiI2CWriteReg8(fd_mpu9250, I2C_SLV0_CTRL, 0x87);

    // Set the AK8963 to continuous measurement mode 1 (8 Hz)
    wiringPiI2CWriteReg8(fd_mpu9250, I2C_SLV0_DO, 0x01);

    // Set the AK8963 control register to write the mode
    wiringPiI2CWriteReg8(fd_mpu9250, I2C_SLV0_REG, AK8963_CNTL1);

    // Enable I2C and set the number of bytes to write
    wiringPiI2CWriteReg8(fd_mpu9250, I2C_SLV0_CTRL, 0x81);

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
    servaddr.sin_addr.s_addr = inet_addr("10.10.1.22"); // IP address, converted to network byte order

    int16_t accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
        
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
                //std::string data = "Angle: " + std::to_string(scan.points[i].angle) + " Range: " + std::to_string(scan.points[i].range) + " Timestamp: " + std::to_string(milliseconds) + "\n";

                // Create a SensorData object and fill it with data
                LIDARData LIDARData;
                LIDARData.angle = scan.points[i].angle; // get angle from lidar
                LIDARData.range = scan.points[i].range; // get range from lidar
                LIDARData.timestamp = milliseconds; // get timestamp

                std::string LIDARDataStr = LIDARDataToString(LIDARData);
                // Send the data over the socket to the specified address and port
                sendto(sockfd, LIDARDataStr.c_str(), LIDARDataStr.size(), 0, (struct sockaddr *)&servaddr, sizeof(servaddr));
            }

            // Read accelerometer data for X, Y and Z axes
            accelX = wiringPiI2CReadReg16(fd_mpu9250, ACCEL_XOUT_H);
            accelX = ((accelX >> 8) & 0xFF) | ((accelX << 8) & 0xFF00);

            accelY = wiringPiI2CReadReg16(fd_mpu9250, ACCEL_YOUT_H);
            accelY = ((accelY >> 8) & 0xFF) | ((accelY << 8) & 0xFF00);

            accelZ = wiringPiI2CReadReg16(fd_mpu9250, ACCEL_ZOUT_H);
            accelZ = ((accelZ >> 8) & 0xFF) | ((accelZ << 8) & 0xFF00);

            // Read gyroscope data for X, Y and Z axes
            gyroX = wiringPiI2CReadReg16(fd_mpu9250, GYRO_XOUT_H);
            gyroX = ((gyroX >> 8) & 0xFF) | ((gyroX << 8) & 0xFF00);

            gyroY = wiringPiI2CReadReg16(fd_mpu9250, GYRO_YOUT_H);
            gyroY = ((gyroY >> 8) & 0xFF) | ((gyroY << 8) & 0xFF00);

            gyroZ = wiringPiI2CReadReg16(fd_mpu9250, GYRO_ZOUT_H);
            gyroZ = ((gyroZ >> 8) & 0xFF) | ((gyroZ << 8) & 0xFF00);

            // Read the magnetometer data
            int16_t magX = wiringPiI2CReadReg16(fd_mpu9250, 0x49);
            int16_t magY = wiringPiI2CReadReg16(fd_mpu9250, 0x4B);
            int16_t magZ = wiringPiI2CReadReg16(fd_mpu9250, 0x4D);

            // Get the current time
            auto now = std::chrono::system_clock::now();
            auto duration = now.time_since_epoch();
            auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

            // Create a SensorData object and fill it with data
            SensorData sensorData;
            sensorData.timestamp = milliseconds; // get timestamp
            sensorData.accelX = accelX; // get accelerometer X axis
            sensorData.accelY = accelY; // get accelerometer Y axis
            sensorData.accelZ = accelZ; // get accelerometer Z axis
            sensorData.gyroX = gyroX; // get gyroscope X axis
            sensorData.gyroY = gyroY; // get gyroscope Y axis
            sensorData.gyroZ = gyroZ; // get gyroscope Z axis
            sensorData.magX = magX; // get magnetometer X axis
            sensorData.magY = magY; // get magnetometer Y axis
            sensorData.magZ = magZ; // get magnetometer Z axis

            std::string sensorDataStr = sensorDataToString(sensorData);
            // Send the data over the socket to the specified address and port
            sendto(sockfd, sensorDataStr.c_str(), sensorDataStr.size(), 0, (struct sockaddr *)&servaddr, sizeof(servaddr));

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