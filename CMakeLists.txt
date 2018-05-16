cmake_minimum_required(VERSION 2.8)
project(robot-identification)

macro(use_cxx11)
    if (CMAKE_VERSION VERSION_LESS "3.1")
        if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
        else ()
            message (FATAL_ERROR "Could not set the compilter standard to C++11.")
        endif ()
    else ()
        set (CMAKE_CXX_STANDARD 11)
    endif ()
endmacro(use_cxx11)
use_cxx11()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

find_package(Eigen3 REQUIRED)

file(GLOB_RECURSE SOURCE_FILES "src/*.cpp")

include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
)

add_executable(axes-ident ${SOURCE_FILES})
