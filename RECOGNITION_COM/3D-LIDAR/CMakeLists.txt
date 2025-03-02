cmake_minimum_required(VERSION 3.25)
project(3D-LIDAR LANGUAGES CXX VERSION 0.0.7)

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)

# Velodyne
find_package(Velodyne REQUIRED)

file(GLOB_RECURSE SOURCE
        src/*.cpp
        src/*.c
)

file(GLOB_RECURSE HEADER
        include/*.hpp
        include/*.h
)

#=========================================================================================

# add library
add_library(${PROJECT_NAME} STATIC ${SOURCE})
target_include_directories(${PROJECT_NAME} PUBLIC
                           include
)
target_link_libraries(${PROJECT_NAME} PUBLIC
        ${OpenCV_LIBS}
        ${Velodyne_LIBS}
)
set_target_properties(${PROJECT_NAME} PROPERTIES
        PUBLIC_HEADER "${PUBLIC_HEADERS}"
        DEBUG_POSTFIX _d)