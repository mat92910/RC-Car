find_package(OpenCV REQUIRED)

add_executable(Lidar-Mapper lidar-mapper.cpp)

#Link your project to ydlidar_sdk library.
target_link_libraries(Lidar-Mapper ${YDLIDAR_SDK_LIBRARIES})
target_link_libraries(Lidar-Mapper ${OpenCV_LIBS})