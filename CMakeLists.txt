cmake_minimum_required(VERSION 3.27)
project(multi_agent)

set(CMAKE_CXX_STANDARD 20)

include_directories(./include)
include_directories(./include/eigen-3.4.0)

add_executable(multi_agent main.cpp
        agent.cpp
        include/agent.h
)
add_executable(test_agent test.cpp
        agent.cpp
        include/agent.h
)
