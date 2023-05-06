cmake_minimum_required(VERSION 3.0)
project(monitor)
file(GLOB SOURCES simple.cpp)
add_executable(monitor ${SOURCES})
