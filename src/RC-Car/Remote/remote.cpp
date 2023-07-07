#include "CYdLidar.h"
#include <thread>
#include <mutex>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <atomic>
#include <asio.hpp>
#include <cmath>

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_sdk.lib")
#endif

using asio::ip::tcp;

std::vector<LaserPoint> points;
std::mutex points_mutex;
bool stop_window_thread = false;

class ConnectionHandler {
public:
    ConnectionHandler(asio::io_service& io_service, const std::string& host, uint16_t port)
        : socket_(io_service), resolver_(io_service) {
        tcp::resolver::query query(host, std::to_string(port));
        tcp::resolver::iterator endpoint_iterator = resolver_.resolve(query);
        asio::connect(socket_, endpoint_iterator);
    }

    void sendData(const std::string& message) {
        asio::write(socket_, asio::buffer(message));
    }

    std::string receiveData() {
        std::array<char, 128> buf;
        std::error_code error;
        size_t len = socket_.read_some(asio::buffer(buf), error);
        if (error == asio::error::eof) {
            return ""; // Connection closed cleanly by peer.
        } else if (error) {
            throw std::system_error(error); // Some other error.
        }
        return std::string(buf.data(), len);
    }

    void disconnect() {
        // Perform a proper shutdown of the socket.
        std::error_code ec;
        socket_.shutdown(tcp::socket::shutdown_both, ec);
        if (ec) {
            throw std::system_error(ec); // Some error occurred.
        }

        // Close the socket.
        socket_.close(ec);
        if (ec) {
            throw std::system_error(ec); // Some error occurred.
        }
    }

private:
    tcp::socket socket_;
    tcp::resolver resolver_;
};

// Function to display LaserPoints in an OpenCV window
void displayLaserPoints(const std::string& window_name, std::atomic<bool>& running) {

    while (!stop_window_thread && running) {
        // Access the LaserPoints safely using a lock_guard
        std::lock_guard<std::mutex> lock(points_mutex);

        // Draw the LaserPoints on the image
        for (const auto& point : points) {
            int x = static_cast<int>(point.range * cos(point.angle) * 100);
            int y = static_cast<int>(point.range * sin(point.angle) * 100);
        }
    }
}

void sendThread(ConnectionHandler& connection_handler) {
    while (true) {
        std::string message;
        std::cout << "Enter message to send: ";
        std::getline(std::cin, message);

        connection_handler.sendData(message);
    }
}

void receiveThread(ConnectionHandler& connection_handler) {
    while (true) {
        std::string data = connection_handler.receiveData();
        std::cout << "Received data: " << data << std::endl;
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

  try {
        const std::string host = "127.0.0.1";
        const uint16_t port = 12345;
        asio::io_service io_service;
        ConnectionHandler connection_handler(io_service, host, port);

        std::thread send_thread(sendThread, std::ref(connection_handler));
        std::thread receive_thread(receiveThread, std::ref(connection_handler));

        // Wait for user input to disconnect.
        std::string input;
        while (std::getline(std::cin, input)) {
            if (input == "disconnect") {
                break;
            }
        }

        // Safely disconnect the client.
        connection_handler.disconnect();

        send_thread.join();
        receive_thread.join();
    } catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

  return 0;
}