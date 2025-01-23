# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

message(STATUS "Building project ${MAKE_PROJECT_NAME}")
message(STATUS "Build for target: ${IDF_BUILD_TARGET}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)


if(NOT DEFINED MAKE_PROJECT_NAME)
    SET(MAKE_PROJECT_NAME example)
endif(NOT DEFINED MAKE_PROJECT_NAME)

project(${MAKE_PROJECT_NAME})
