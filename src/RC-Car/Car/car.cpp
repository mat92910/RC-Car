#include "CYdLidar.h"
#include <thread>
#include <mutex>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <atomic>
#include <asio.hpp>

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_sdk.lib")
#endif

using asio::ip::tcp;

class Session {
public:
    Session(tcp::socket socket)
        : socket_(std::move(socket)) {
    }

    void start() {
        std::thread receive_thread(&Session::receiveData, this);
        std::thread send_thread(&Session::sendData, this);

        receive_thread.join();
        send_thread.join();
    }

private:
    void receiveData() {
        while (true) {
            std::array<char, 128> buf;
            std::error_code error;
            size_t len = socket_.read_some(asio::buffer(buf), error);

            if (error == asio::error::eof) {
                break; // Connection closed cleanly by peer.
            } else if (error) {
                throw std::system_error(error); // Some other error.
            }

            std::string data(buf.data(), len);
            std::cout << "Received data: " << data << std::endl;
        }
    }

    void sendData() {
        while (true) {
            std::string message;
            std::cout << "Enter message to send: ";
            std::getline(std::cin, message);

            asio::write(socket_, asio::buffer(message));
        }
    }

    tcp::socket socket_;
};

class Server {
public:
    Server(asio::io_service& io_service, uint16_t port)
        : acceptor_(io_service, tcp::endpoint(tcp::v4(), port)) {
        accept();
    }

private:
    void accept() {
        acceptor_.async_accept([this](std::error_code ec, tcp::socket socket) {
            if (!ec) {
                std::make_shared<Session>(std::move(socket))->start();
            }

            accept();
        });
    }

    tcp::acceptor acceptor_;
};

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

  try {
      uint16_t port = 12345;
      asio::io_service io_service;
      Server server(io_service, port);

      io_service.run();
  } catch (std::exception& e) {
      std::cerr << "Exception: " << e.what() << std::endl;
  }

  return 0;
}