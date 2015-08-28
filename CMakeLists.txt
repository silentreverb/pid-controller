cmake_minimum_required(VERSION 2.6)
project(PIDController)
find_package(Boost REQUIRED COMPONENTS system chrono timer)
include_directories(include)
add_library(pid-controller SHARED
    src/PIDController.cpp
)
target_link_libraries(pid-controller
    ${Boost_LIBRARIES}
)
install(TARGETS pid-controller DESTINATION lib)
install(FILES include/PIDController.h DESTINATION include)
