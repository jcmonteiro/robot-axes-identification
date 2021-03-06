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
        set(CMAKE_CXX_STANDARD 11)
        set(CMAKE_CXX_STANDARD_REQUIRED ON)
        set(CMAKE_CXX_EXTENSIONS OFF)
    endif ()
endmacro(use_cxx11)
use_cxx11()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS unit_test_framework)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
)

add_library(axes-ident SHARED "src/DataParser.cpp" "src/Identification.cpp")

# Unit tests
if (Boost_FOUND)
    ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK) 
    add_executable(test-ident tests/test_Identification.cpp)
    target_include_directories(test-ident PRIVATE ${Boost_INCLUDE_DIRS})
    target_link_libraries(test-ident ${Boost_LIBRARIES} axes-ident)
    add_test(NAME test1 COMMAND test-ident)
endif ()
