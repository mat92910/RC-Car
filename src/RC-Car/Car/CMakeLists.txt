EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )

if(NOT ${ARCHITECTURE} STREQUAL "x86_64" )
    add_executable(Car car.cpp)

    #Link your project to the libraries.
    target_link_libraries(Car ${YDLIDAR_SDK_LIBRARIES})
    target_link_libraries(Car ${WIRINGPI_LIBRARIES}) 
endif()
