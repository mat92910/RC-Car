add_executable(Lidar-Mapper lidar-mapper.cpp)

#Link your project to the libraries.
target_link_libraries(Lidar-Mapper ${YDLIDAR_SDK_LIBRARIES})
target_link_libraries(Lidar-Mapper ${OpenCV_LIBS})