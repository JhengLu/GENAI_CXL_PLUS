cmake_minimum_required(VERSION 3.22)
project(monitor)
set(CMAKE_CXX_STANDARD 20)

# Find Python 3 from the environment
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# Include Python headers
include_directories(${Python3_INCLUDE_DIRS})

# Add the executable
add_executable(monitor_bw monitor_bw.cpp)
add_executable(monitor monitor.cpp)

# Link the executable with Python libraries
target_link_libraries(monitor_bw Python3::Python)
target_link_libraries(monitor Python3::Python)

# Add the second executable
add_executable(monitor_new monitor_new.cpp)

