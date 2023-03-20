#include "CYdLidar.h"
#include <string>
using namespace std;
using namespace ydlidar;

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_sdk.lib")
#endif

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
  printf("__   ______  _     ___ ____    _    ____  \n");
  printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\ \n");
  printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) | \n");
  printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <  \n");
  printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\ \n");
  printf("\n");
  fflush(stdout);

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

  // Turn On success and loop
  while (ret && ydlidar::os_isOk())
  {
    LaserScan scan;
    if (laser.doProcessSimple(scan))
    {
      fprintf(stdout, "Scan received[%llu]: %u ranges is [%f]Hz\n",
              scan.stamp,
              (unsigned int)scan.points.size(), 1.0 / scan.config.scan_time);
      fflush(stdout);
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