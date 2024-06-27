cmake_minimum_required(VERSION 3.5)
project(test)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(Corrector)
include_directories(
    Corrector/include
)
add_executable(main main.cpp)

target_link_libraries(main
    correct
)