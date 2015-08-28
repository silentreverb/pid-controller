cmake_minimum_required(VERSION 2.6)
project(PIDexample)
find_package(Boost REQUIRED COMPONENTS system timer chrono)
set (PROJECT_LINK_LIBS libpid-controller.so)
include_directories(include)
add_executable(PIDexample PIDexample.cpp)
target_link_libraries(PIDexample
    ${PROJECT_LINK_LIBS}
    ${Boost_LIBRARIES}
)
