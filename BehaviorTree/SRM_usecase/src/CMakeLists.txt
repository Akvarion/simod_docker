cmake_minimum_required(VERSION 3.5)
project(SRM1)

set(CMAKE_CXX_STANDARD 17)

find_package(behaviortree_cpp REQUIRED)

add_executable(SRM1 main.cpp)

target_link_libraries(SRM1 BT::behaviortree_cpp)