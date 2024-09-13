cmake_minimum_required(VERSION 3.25)
project(PointCloud Studio)
set(CMAKE_CXX_STANDARD 20)

# 设置输出目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/bin")

# MSCV设置UTF-8
add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")

# 包含子项目
add_subdirectory("src/PCLSharp.Primitives.Native")
add_subdirectory("src/PCLSharp.Modules.Native")
