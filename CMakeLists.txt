cmake_minimum_required(VERSION 3.10)
project(SICKTest)

find_package(OpenCV REQUIRED)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(test test.cpp)

target_link_libraries(test
    ${OpenCV_LIBS}
    sick_safetyscanners_base
)


#target_link_libraries(test sick_safetyscanners_base)
