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
    if(cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) != -1) {
      cv::destroyWindow(window_name);
    }
}

int main(int argc, char *argv[])
{
  std::cout <<  "\033[1;35m"
                " _____                      _       \n"
                "|  __ \\                    | |      \n"
                "| |__) |___ _ __ ___   ___ | |_ ___ \n"
                "|  _  // _ \\ '_ ` _ \\ / _ \\| __/ _ \\\n"
                "| | \\ \\  __/ | | | | | (_) | ||  __/\n"
                "|_|  \\_\\___|_| |_| |_|\\___/ \\__\\___|\n"
                "\033[0m" << std::endl;

  // This flag will be used to control the loop
  std::atomic<bool> running(true);

  // Start the displayLaserPoints thread
  std::thread window_thread(displayLaserPoints, "Laser Points", std::ref(running));

  while (running)
  {
    // Update the LaserPoints
    
  }

  // Stop the window_thread
  stop_window_thread = true;
  // Wait for the window_thread to finish
  window_thread.join();
  return 0;
}