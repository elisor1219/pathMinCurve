# Set the minimum required version of CMake
cmake_minimum_required(VERSION 3.0)

# Set the project name
project(Curve_Minimization)

# Add the source files to the project
add_executable(minCurvOpti.out minCurvOpti.cpp curvature_minimizer.cpp)

# Set the C++ standard to C++11
set(CMAKE_CXX_STANDARD 11)

# Include the header files
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Set any necessary compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lstdc++ -O3")

# Find the NLopt package
find_package(NLopt REQUIRED)

# Link the NLopt library to the target
target_link_libraries(minCurvOpti.out PRIVATE NLopt::nlopt)
