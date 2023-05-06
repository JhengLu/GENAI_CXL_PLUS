cmake_minimum_required(VERSION 3.0)
project(monitor)
file(GLOB SOURCES monitor.cpp)
add_executable(monitor ${SOURCES})
