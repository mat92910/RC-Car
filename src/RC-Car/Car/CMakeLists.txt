find_package(OpenCV REQUIRED)

add_executable(Car car.cpp)

#Link your project to ydlidar_sdk library.
target_link_libraries(Car ${YDLIDAR_SDK_LIBRARIES})
target_link_libraries(Car ${OpenCV_LIBS})