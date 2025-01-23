# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

message(STATUS "Building project ${MAKE_PROJECT_NAME}")
message(STATUS "Build for target: ${IDF_BUILD_TARGET}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)


if(NOT DEFINED MAKE_PROJECT_NAME)
    SET(MAKE_PROJECT_NAME devcontainer-esp-idf-ds18b20)
endif(NOT DEFINED MAKE_PROJECT_NAME)

project(${MAKE_PROJECT_NAME})

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_C_STANDARD 23)

add_compile_options(
    -std=c++23
    -Wall
    -Wextra
    -Wconversion
    -Wshadow
    -Wpedantic
    -Wnarrowing
    -Wimplicit
    -Waddress
    -pedantic
    -Wdeprecated
    -Wregister
    -Wnon-virtual-dtor
    -Wpessimizing-move
    -Wsign-conversion
    -Wduplicated-cond
    -Wduplicated-branches
    -Wlogical-op
    -Wnull-dereference
    -Wuseless-cast
    -Wdouble-promotion
    -Wimplicit-fallthrough
    -Wcast-align
    -fconcepts
)
