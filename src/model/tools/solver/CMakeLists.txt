#check requirements
cmake_minimum_required(VERSION "3.0")

set(CMAKE_CXX_COMPILER:FILEPATH C:/MinGW/bin/gcc.exe)
set(CMAKE_C_COMPILER:FILEPATH C:/MinGW/bin/gcc.exe)

#settings
#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-O3")

#project name
project(solver)

add_executable(solver "main.cpp")