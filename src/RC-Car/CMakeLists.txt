find_package(OpenCV REQUIRED)

add_executable(RC-Car rc-car.cpp)

#Link your project to ydlidar_sdk library.
target_link_libraries(RC-Car ${YDLIDAR_SDK_LIBRARIES})
target_link_libraries(RC-Car ${OpenCV_LIBS})