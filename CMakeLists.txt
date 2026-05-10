########################################################################
# Build Soapy SDR support module for Sidekiq Devices
########################################################################
cmake_minimum_required(VERSION 3.16)
project(SoapySidekiq CXX)

# SoapySDR modules are loaded via dlopen, so every translation unit that
# ends up in the .so must be compiled position-independent. Set this
# before any targets are declared so it propagates to everything below.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SoapySDR "0.4.0" NO_MODULE REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package(Sidekiq REQUIRED)

message(STATUS "Sidekiq_INCLUDE_DIRS - ${Sidekiq_INCLUDE_DIRS}")
message(STATUS "Sidekiq_LIBRARIES    - ${Sidekiq_LIBRARIES}")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

if(CMAKE_COMPILER_IS_GNUCXX)
    add_compile_options(-Wno-unused-parameter)
endif()

SOAPY_SDR_MODULE_UTIL(
    TARGET SidekiqSupport
    SOURCES
        SoapySidekiq.hpp
        Registration.cpp
        Settings.cpp
        Streaming.cpp
        Sensor.cpp
    LIBRARIES
        Sidekiq::Sidekiq
)
