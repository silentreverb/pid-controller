cmake_minimum_required(VERSION 2.6)
project(PIDexample)
find_package(Boost REQUIRED COMPONENTS system timer chrono)
include_directories(include)
add_library(pid-controller SHARED IMPORTED
  /usr/local/lib/libpid-controller.so
)
set_target_properties(pid-controller PROPERTIES
    IMPORTED_LOCATION /usr/local/lib/libpid-controller.so
)
add_executable(PIDexample src/PIDexample.cpp)
target_link_libraries(PIDexample
    pid-controller
    ${Boost_LIBRARIES}
)
