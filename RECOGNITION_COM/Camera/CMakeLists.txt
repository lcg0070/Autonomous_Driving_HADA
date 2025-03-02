cmake_minimum_required(VERSION 3.25)
project(Camera LANGUAGES CXX CUDA VERSION 0.0.7)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(YOLO)

file(GLOB_RECURSE SOURCE
     src/*.cpp
     src/*.c
     )

file(GLOB_RECURSE HEADER
     include/*.hpp
     include/*.h
     )


#   LIBRARY

add_library(${PROJECT_NAME} STATIC ${SOURCE} ${HEADER})
add_dependencies(${PROJECT_NAME} YOLO)
target_include_directories(${PROJECT_NAME} PUBLIC
                           ${CUDA_INCLUDE_DIRS}
                           ${CMAKE_CURRENT_SOURCE_DIR}/include
                           )
target_link_libraries(${PROJECT_NAME} PUBLIC
                      ${OpenCV_LIBS}
                      ${Communication_LIBS}
                      YOLO
                      3D-LIDAR
                      )
set_target_properties(${PROJECT_NAME} PROPERTIES
                      PUBLIC_HEADER "${PUBLIC_HEADERS}"
                      DEBUG_POSTFIX _d
                      )

file(COPY etc DESTINATION ${CMAKE_BINARY_DIR})