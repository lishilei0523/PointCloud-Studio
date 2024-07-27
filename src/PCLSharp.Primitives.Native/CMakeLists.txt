cmake_minimum_required(VERSION 3.25)
project(PCLSharp.Primitives.Native)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB INCLUDE_FILES
    "${CMAKE_CURRENT_LIST_DIR}/*.h")
file(GLOB SOURCE_FILES
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp")

add_library(${PROJECT_NAME} SHARED ${INCLUDE_FILES} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

# MSVC大文件编译选项
target_compile_options(${PROJECT_NAME} PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/bigobj>)
