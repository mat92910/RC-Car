#include "CYdLidar.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>
#include <mutex>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <atomic>

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_sdk.lib")
#endif

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

int main(int argc, char *argv[])
{
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

  // This flag will be used to control the loop
  std::atomic<bool> running(true);

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

  while (ret && ydlidar::os_isOk() && running)
  {
    LaserScan scan;
    if (laser.doProcessSimple(scan))
    {
      // Update the LaserPoints
      
    }
    else
    {
      fprintf(stderr, "Failed to get Lidar Data\n");
      fflush(stderr);
    }
  }

  // Stop the device scanning thread and disable motor.
  laser.turnOff();
  // Uninitialize the SDK and Disconnect the LiDAR.
  laser.disconnecting();
  return 0;
}