cmake_minimum_required(VERSION 3.7)
project(robot-identification)

find_package(Eigen3 REQUIRED)

file(GLOB_RECURSE SOURCE_FILES "src/*.cpp")

include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
)

add_executable(axes-ident ${SOURCE_FILES})
