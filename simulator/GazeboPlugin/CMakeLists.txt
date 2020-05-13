#/* Copyright 2019 The MathWorks, Inc. */
##########################################
# CMakeLists.txt for gazebo_plugin_for_simulink #
##########################################

cmake_minimum_required(VERSION 2.8)

PROJECT(gazebo_plugin_for_simulink)

#################################
# Specify the CMake module path #
#################################

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++ -Wno-deprecated-declarations -Wno-unused-function")
ENDIF()

find_package(Boost 1.58.0 REQUIRED COMPONENTS date_time)
find_package(gazebo 9 REQUIRED)

include_directories(
include/
msgsproto/
${Boost_INCLUDE_DIR}
${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

ADD_SUBDIRECTORY(msgsproto)
ADD_SUBDIRECTORY(src/transport)
ADD_SUBDIRECTORY(src/gazeboserver)
ADD_SUBDIRECTORY(src/gazeboplugin)

###############################################
# USE => cmake .. -DTESTING=ON  to trun ON testing

option(TESTING "plugin testing" OFF)

if(TESTING)
   ADD_SUBDIRECTORY(src/gazeboplugin/pkgtest)
   ADD_SUBDIRECTORY(src/gazeboserver/gazebomsghandler/pkgtest)
endif()

