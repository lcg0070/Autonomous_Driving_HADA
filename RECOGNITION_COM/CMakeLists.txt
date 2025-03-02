cmake_minimum_required(VERSION 3.25)
project(RECOGNITION LANGUAGES C CXX CUDA VERSION 0.0.3)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(Communication REQUIRED)
find_package(CUDA REQUIRED)

add_subdirectory(3D-LIDAR)
add_subdirectory(Camera)
add_subdirectory(OFF)

add_dependencies(Camera 3D-LIDAR)
add_dependencies(Camera OFF)

file(GLOB_RECURSE SOURCE
        src/*.c
        src/*.cpp
)

file(GLOB_RECURSE HEADER
        include/*.h
        include/*.hpp
)

add_executable(${PROJECT_NAME}
        RECOG2CONTROL.cpp
        ${SOURCE}
        ${HEADER}
)

add_dependencies(${PROJECT_NAME} OFF Camera)

target_link_libraries(${PROJECT_NAME} PUBLIC
        ${Communication_LIBS}
        OFF
        Camera
)

target_include_directories(${PROJECT_NAME} PUBLIC include)
