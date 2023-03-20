add_executable(RC-Car rc-car.cpp)

#Link your project to ydlidar_sdk library.
target_link_libraries(RC-Car ${YDLIDAR_SDK_LIBRARIES})