find_package(OpenCV REQUIRED)

add_executable(LiDAR-Display lidar-display.cpp)

#Link your project to ydlidar_sdk library.
target_link_libraries(LiDAR-Display ${YDLIDAR_SDK_LIBRARIES})
target_link_libraries(LiDAR-Display ${OpenCV_LIBS})