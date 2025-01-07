cmake_minimum_required(VERSION 3.5)
project(BallGrasp)

set(CMAKE_CXX_STANDARD 17)

find_package(behaviortree_cpp REQUIRED)

add_executable(BallGrasp main.cpp)

target_link_libraries(BallGrasp BT::behaviortree_cpp)