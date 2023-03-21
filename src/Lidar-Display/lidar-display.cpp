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

std::vector<LaserPoint> points;
std::mutex points_mutex;
bool stop_window_thread = false;

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

// Function to display LaserPoints in an OpenCV window
void displayLaserPoints(const std::string& window_name, std::atomic<bool>& running) {
    int window_width = 800;
    int window_height = 800;

    // Create an OpenCV window
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

    while (!stop_window_thread && running) {
        // Create a black image with specified dimensions and 3 color channels
        cv::Mat img(window_height, window_width, CV_8UC3, cv::Scalar(0, 0, 0));

        // Access the LaserPoints safely using a lock_guard
        std::lock_guard<std::mutex> lock(points_mutex);

        // Draw the LaserPoints on the image
        for (const auto& point : points) {
            int x = static_cast<int>(point.range * std::cos(point.angle) * 100) + window_width / 2;
            int y = static_cast<int>(point.range * std::sin(point.angle) * 100) + window_height / 2;

            cv::circle(img, cv::Point(x, y), 1, cv::Scalar(0, 255, 0), -1);
        }

        // Calculate the center of the image
        cv::Point center(window_width / 2, window_height / 2);

        // Calculate the top-left and bottom-right points of the square
        cv::Point top_left(center.x - 10 / 2, center.y - 20 / 2);
        cv::Point bottom_right(center.x + 10 / 2, center.y + 20 / 2);

        // Draw the square on the image
        cv::rectangle(img, top_left, bottom_right, cv::Scalar(0, 0, 255), -1);

        // Display the image in the window
        cv::imshow(window_name, img);

        // Wait for a key press or 30 ms, whichever comes first
        int key = cv::waitKey(30);

        if (key == 27 || cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) == -1) {
          running = false; // Set the running flag to false to stop the loop in the main thread
        }
    }

    // Close the window
    cv::destroyWindow(window_name);
}

int main(int argc, char *argv[])
{
  std::cout <<  "\033[1;35m"
                " _      _     _                       _____  _           _             \n"
                "| |    (_)   | |                     |  __ \\(_)         | |            \n"
                "| |     _  __| | __ _ _ __   ______  | |  | |_ ___ _ __ | | __ _ _   _ \n"
                "| |    | |/ _` |/ _` | '__| |______| | |  | | / __| '_ \\| |/ _` | | | |\n"
                "| |____| | (_| | (_| | |             | |__| | \\__ \\ |_) | | (_| | |_| |\n"
                "|______|_|\\__,_|\\__,_|_|             |_____/|_|___/ .__/|_|\\__,_|\\__, |\n"
                "                                                  | |             __/ |\n"
                "                                                  |_|            |____/ \n"
                "\033[0m" << std::endl;

  // init system signal
  ydlidar::os_init();

  CYdLidar laser;

  InitializeLidar(laser);

  // This flag will be used to control the loop
  std::atomic<bool> running(true);

  // Start the displayLaserPoints thread
  std::thread window_thread(displayLaserPoints, "Laser Points", std::ref(running));

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
      {
          std::lock_guard<std::mutex> lock(points_mutex);
          points = scan.points;
      }
    }
    else
    {
      fprintf(stderr, "Failed to get Lidar Data\n");
      fflush(stderr);
    }
  }

  // Wait for the input_thread to finish before exiting the program
  //input_thread.join();
  // Stop the window_thread
  stop_window_thread = true;
  // Wait for the window_thread to finish
  window_thread.join();
  // Stop the device scanning thread and disable motor.
  laser.turnOff();
  // Uninitialize the SDK and Disconnect the LiDAR.
  laser.disconnecting();
  return 0;
}