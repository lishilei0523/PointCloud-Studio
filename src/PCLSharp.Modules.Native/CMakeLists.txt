cmake_minimum_required(VERSION 3.25)
project(PCLSharp.Modules.Native)
set(CMAKE_CXX_STANDARD 20)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories("../PCLSharp.Primitives.Native")

file(GLOB INCLUDE_FILES
    "${CMAKE_CURRENT_LIST_DIR}/*.h")

file(GLOB SOURCE_FILES
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp")

add_library(PCLSharp.Modules.Native SHARED ${INCLUDE_FILES} ${SOURCE_FILES})

target_link_libraries(PCLSharp.Modules.Native ${PCL_LIBRARIES})
target_link_libraries(PCLSharp.Modules.Native PCLSharp.Primitives.Native)

target_compile_options(PCLSharp.Modules.Native 
	PRIVATE 
	$<$<CXX_COMPILER_ID:MSVC>:/bigobj> 
	$<$<CXX_COMPILER_ID:GNU>:-Wa,-mbig-obj>) 
