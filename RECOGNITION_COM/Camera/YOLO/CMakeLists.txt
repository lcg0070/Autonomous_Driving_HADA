cmake_minimum_required(VERSION 3.25)
project(YOLO LANGUAGES CUDA VERSION 0.3.2)

set(CMAKE_CUDA_STANDARD 17)

#-------------------------------------------------- Build Initialize --------------------------------------------------#

# Requires registration in the Path with the name ‘TensorRT_DIR’
set(TensorRT_ROOT $ENV{TensorRT_DIR})

if (NOT TensorRT_ROOT)
    message(WARNING "TensorRT path is incorrect.")
endif ()

message(STATUS "TensorRT Root: " ${TensorRT_ROOT})

# Find TensorRT include directory
find_path(TensorRT_INCLUDE_DIRS
          NAMES NvInfer.h
          HINTS ${TensorRT_ROOT} ${CUDA_TOOLKIT_ROOT_DIR} $ENV{Path}
          PATH_SUFFIXES include)

if (NOT TensorRT_INCLUDE_DIRS)
    message(FATAL_ERROR "* Can Not found TensorRT header")
endif ()

message(STATUS "Found TensorRT header")
message(STATUS "Tensor Include Root: " ${TensorRT_INCLUDE_DIRS})
message("")

# Find TensorRT Library
find_library(TensorRT_LIBRARY
             NAMES nvinfer_10
             HINTS ${TensorRT_ROOT} ${CUDA_TOOLKIT_ROOT_DIR} $ENV{Path}
             PATH_SUFFIXES lib lib64 lib/x64)

if (NOT TensorRT_LIBRARY)
    message(FATAL_ERROR "Can Not found TensorRT libraries")
endif ()

message(STATUS "Found TensorRT libraries")
message(STATUS "Tensor libraries: " ${TensorRT_LIBRARY})
message("")

# Find ONNX parser library
find_library(TensorRT_ONNXPARSER_LIBRARY
             NAMES nvonnxparser_10
             HINTS ${TensorRT_ROOT} ${CUDA_TOOLKIT_ROOT_DIR} $ENV{Path}
             PATH_SUFFIXES lib lib64 lib/x64)

if (NOT TensorRT_ONNXPARSER_LIBRARY)
    message(FATAL_ERROR "Can Not found TensorRT ONNX parser libraries")
endif ()

message(STATUS "Found TensorRT parser libraries")
message(STATUS "TensorRT ONNX parser libraries: " ${TensorRT_ONNXPARSER_LIBRARY})
message("")

# Group TensorRT libraries
set(TensorRT_LIBRARIES
    ${TensorRT_LIBRARY}
    ${TensorRT_ONNXPARSER_LIBRARY})

if (${CMAKE_BUILD_TYPE} MATCHES Debug)
    find_library(Engine_LIB
                 NAMES Engine_d
                 HINTS ${PROJECT_SOURCE_DIR}/TensorRT-API
                 PATH_SUFFIXES lib
                 )
else ()
    find_library(Engine_LIB
                 NAMES Engine
                 HINTS ${PROJECT_SOURCE_DIR}/TensorRT-API
                 PATH_SUFFIXES lib
                 )
endif ()

#------------------------------------------------ Build Configuration -------------------------------------------------#

# CUDA 유틸 라이브러리
file(GLOB_RECURSE SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cu)
set(PUBLIC_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/YOLO.cuh
    ${CMAKE_CURRENT_SOURCE_DIR}/TensorRT-API/include/Engine.cuh
    )

add_library(${PROJECT_NAME} STATIC ${SOURCE})
target_include_directories(${PROJECT_NAME} PUBLIC
                           ${CMAKE_CURRENT_SOURCE_DIR}/include
                           ${CUDA_INCLUDE_DIRS}
                           ${TensorRT_INCLUDE_DIRS}
                           ${PROJECT_SOURCE_DIR}/TensorRT-API/include
                           )
target_link_libraries(${PROJECT_NAME} PUBLIC
                      ${OpenCV_LIBS}
                      ${TensorRT_LIBRARIES}
                      ${Engine_LIB}
                      )
set_target_properties(${PROJECT_NAME} PROPERTIES
                      PUBLIC_HEADER "${PUBLIC_HEADERS}"
                      DEBUG_POSTFIX _d)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_PREFIX}/include
        )

install(FILES ${Engine_LIB}
        DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
        )
