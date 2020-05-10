cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
PROJECT(ManualSFM)

SET (CMAKE_CXX_STANDARD 14)

include_directories(src)

add_subdirectory(src)
add_subdirectory(tst)
add_subdirectory(lib/googletest)