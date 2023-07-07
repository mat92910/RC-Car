add_executable(Car car.cpp)

#Link your project to the libraries.
target_link_libraries(Car ${YDLIDAR_SDK_LIBRARIES})