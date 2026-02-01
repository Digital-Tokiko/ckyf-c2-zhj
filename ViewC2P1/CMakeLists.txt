cmake_minimum_required(VERSION 4.1)
project(ViewC2P1)

set(CMAKE_CXX_STANDARD 20)

include_directories(Class)

add_executable(ViewC2P1 main.cpp Class/Command.cpp Class/Robot.cpp Class/RobotManager.cpp)


