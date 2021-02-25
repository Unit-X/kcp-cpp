cmake_minimum_required(VERSION 3.10)
project(kcp_cpp)

set(CMAKE_CXX_STANDARD 17)
find_package (Threads REQUIRED)

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
        GIT_TAG 77a4f85ae69f03fd9008b799ee99bb1f098befc5
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

IF (WIN32)
    set_property(TARGET kcp PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/kcp/${CMAKE_BUILD_TYPE}/kcp.lib)
ELSE()
    set_property(TARGET kcp PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/kcp/libkcp.a)
ENDIF()

add_dependencies(kcp project_kcp)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/kcp/)

add_library(kcpnet STATIC KCPNet.cpp)
target_link_libraries(kcpnet kcp Threads::Threads)

add_executable(kcp_cpp main.cpp)
target_link_libraries(kcp_cpp kcpnet Threads::Threads)