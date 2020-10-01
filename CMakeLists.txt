cmake_minimum_required(VERSION 3.17)
project(kcp_cpp)

set(CMAKE_CXX_STANDARD 17)

#If no build type is set then force Release
IF( NOT CMAKE_BUILD_TYPE )
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
            "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
            FORCE)
ENDIF()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")

#Include KCP
include(ExternalProject)
ExternalProject_Add(project_kcp
        GIT_REPOSITORY https://github.com/skywind3000/kcp.git
        GIT_SUBMODULES ""
        UPDATE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/kcp
        BINARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/kcp
        GIT_PROGRESS 1
        BUILD_COMMAND cmake --build ${CMAKE_CURRENT_SOURCE_DIR}/kcp --config ${CMAKE_BUILD_TYPE} --target kcp
        STEP_TARGETS build
        EXCLUDE_FROM_ALL TRUE
        INSTALL_COMMAND ""
        )
add_library(kcp STATIC IMPORTED)
set_property(TARGET kcp PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/kcp/libkcp.a)
add_dependencies(kcp project_kcp)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/kcp/)

add_library(kcpnet STATIC kcpnet.cpp)

add_executable(kcp_cpp main.cpp)
target_link_libraries(kcp_cpp kcp kcpnet)