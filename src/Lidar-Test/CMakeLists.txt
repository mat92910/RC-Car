add_executable(LiDAR-Test lidar-test.cpp)

#Link your project to ydlidar_sdk library.
target_link_libraries(LiDAR-Test ${YDLIDAR_SDK_LIBRARIES})