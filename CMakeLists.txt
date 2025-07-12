# File: CMakeLists.txt (root directory) - ESP32-S3 Compatible

cmake_minimum_required(VERSION 3.16)

# Set the target to ESP32-S3 before including ESP-IDF
set(IDF_TARGET esp32s3)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(trolley4_s3)