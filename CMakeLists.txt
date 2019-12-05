cmake_minimum_required(VERSION 3.13.4)

project(OpenCVTest)

option(CMAKE_EXPORT_COMPILE_COMMANDS "Generate compile_commands.json in build directory" ON)

find_package(OpenCV REQUIRED)
add_executable(OpenCVTest src/CameraCalibration.cpp)
target_link_libraries(OpenCVTest ${OpenCV_LIBS})
