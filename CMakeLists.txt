cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include(pico_sdk_import.cmake)

project(pico_dps310 C CXX ASM)

pico_sdk_init()

set(TARGET_GROUP test CACHE STRING "Group to build")
if(TARGET_GROUP STREQUAL production)
    add_subdirectory("src")
elseif(TARGET_GROUP STREQUAL test)
    include(CTest)

    add_subdirectory("src")
    add_subdirectory("external")
    add_subdirectory("test")
    add_subdirectory("examples")
else()
    message(FATAL_ERROR "Given TARGET_GROUP unknown")
endif()
