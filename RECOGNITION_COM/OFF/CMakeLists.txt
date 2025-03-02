cmake_minimum_required(VERSION 3.25.0)
project(OFF LANGUAGES C CXX VERSION 0.1.1)

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)

# file include, src
file(GLOB_RECURSE SOURCE
        src/*.c
        src/*.cpp

        LSH/src/*.c
        LSH/src/*.cpp
)
file(GLOB_RECURSE HEADER
        include/*.h
        include/*.hpp

        LSH/include/*.h
        LSH/include/*.hpp
)

# OpenCV 설정
#find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} include)
link_libraries(${OpenCV_LIBS})

#2D-Lidar library
find_package(2D_LIDAR REQUIRED)

#Communication
find_package(Communication REQUIRED)


add_library(${PROJECT_NAME} STATIC
        ${SOURCE}
        ${HEADER}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
        ${OpenCV_LIBS}
        ${2D_LIDAR_LIBS}
        ${Communication_LIBS}
)
target_include_directories(${PROJECT_NAME} PUBLIC
        include
        LSH/include
)
