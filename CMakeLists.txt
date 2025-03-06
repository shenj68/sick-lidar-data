cmake_minimum_required(VERSION 3.10)
project(SICKTest)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

add_executable(test test.cpp)
target_link_libraries(test sick_safetyscanners_base)
