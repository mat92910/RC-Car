cmake_minimum_required(VERSION 3.0.0)
project(RC-Car VERSION 0.1.0)

#include(CTest)
#enable_testing()

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)


#find ydlidar_sdk package
find_package(ydlidar_sdk REQUIRED)
#find OpenCV Package
find_package(OpenCV REQUIRED)
if( ${ARCHITECTURE} STREQUAL "64Bit" )
    #find WiringPi library
    find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
    include_directories(${WIRINGPI_INCLUDE_DIRS}) 
endif()


#Include directories
include_directories(${YDLIDAR_SDK_INCLUDE_DIRS}) 
add_subdirectory(src)

#link library directories
link_directories(${YDLIDAR_SDK_LIBRARY_DIRS})
