cmake_minimum_required(VERSION 3.10)
project(esp32_systemc)

set(CMAKE_CXX_STANDARD 17)

set(SYSTEMC_HOME "/opt/systemc")
include_directories(${SYSTEMC_HOME}/include)
link_directories(${SYSTEMC_HOME}/lib)

file(GLOB SOURCES "src/*.cpp")
add_executable(simulation ${SOURCES})
target_link_libraries(simulation systemc)
