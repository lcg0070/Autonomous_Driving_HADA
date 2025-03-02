cmake_minimum_required(VERSION 3.25)
project(3D-LIDAR LANGUAGES CXX VERSION 0.0.7)

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)

# OpenCV
find_package(OpenCV REQUIRED PATHS $ENV{OpenCV_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBS})

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
# normal test
add_executable(calibration_main main.cpp
        ${SOURCE}
        ${HEADER}
)
target_include_directories(calibration_main PRIVATE
        include
)
target_link_libraries(calibration_main
        ${Velodyne_LIBS}
)
#=========================================================================================

#=========================================================================================
# visual test
add_executable(3D_visual_test visualize_test.cpp
        ${SOURCE}
        ${HEADER}
)
target_include_directories(3D_visual_test PRIVATE
        include
)
target_link_libraries(3D_visual_test
        ${Velodyne_LIBS}
)
#=========================================================================================

# add library
add_library(${PROJECT_NAME} STATIC ${SOURCE})
target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(${PROJECT_NAME} PUBLIC
        ${OpenCV_LIBS}
        ${Velodyne_LIBS}
)
set_target_properties(${PROJECT_NAME} PROPERTIES
        PUBLIC_HEADER "${PUBLIC_HEADERS}"
        DEBUG_POSTFIX _d)