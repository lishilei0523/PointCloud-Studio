cmake_minimum_required(VERSION 3.25)
project(PCLSharp.Modules.Native)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories("../PCLSharp.Primitives.Native")

file(GLOB INCLUDE_FILES
    "${CMAKE_CURRENT_LIST_DIR}/*.h")
file(GLOB SOURCE_FILES
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp")

add_library(${PROJECT_NAME} SHARED ${INCLUDE_FILES} ${SOURCE_FILES} ${CMAKE_CURRENT_LIST_DIR}/Version.rc)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} "PCLSharp.Primitives.Native")

# MSVC大文件编译选项
target_compile_options(${PROJECT_NAME} PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/bigobj>)

# 添加版本信息
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION 1.0.1 SOVERSION 1.0.1)
