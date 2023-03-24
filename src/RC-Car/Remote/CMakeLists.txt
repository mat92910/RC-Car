add_executable(Remote remote.cpp)

#Link your project to the libraries.
target_link_libraries(Remote ${YDLIDAR_SDK_LIBRARIES})
target_link_libraries(Remote ${OpenCV_LIBS})