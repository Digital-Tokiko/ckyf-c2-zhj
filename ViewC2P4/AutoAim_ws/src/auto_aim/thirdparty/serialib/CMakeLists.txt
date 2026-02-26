#cmake_minimum_required(VERSION 3.23)
cmake_minimum_required(VERSION 3.8)
project(serialib)

set(CMAKE_CXX_STANDARD 14)

add_library(serialib serialib.cpp)
