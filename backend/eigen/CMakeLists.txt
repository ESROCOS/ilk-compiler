cmake_minimum_required(VERSION 2.8)
project(ilk_eigen_backend)

find_package(PkgConfig)
pkg_check_modules(EIGEN3 eigen3 REQUIRED)

set(HEADER_INSTALL_ROOT /usr/local/include/ CACHE PATH "Where to install header files")
set(LIB_INSTALL_ROOT    /usr/local/lib/     CACHE PATH "Where to install the shared library")

# Flags
set(CMAKE_CXX_FLAGS "-g -Wall -O3 -D EIGEN_NO_DEBUG -std=c++11")
include_directories(${EIGEN3_INCLUDE_DIRS})



set(HEADERS
  ./core-types.h
  ./joint-transforms.h
  ./gjac.h
  ./rots.h
  ./ik.h
  ./misc.h
  ./test-utils.h
)
set(SOURCES
  ./joint-transforms.cpp
  ./test-utils.cpp
)

#
# Add library and install targets
#
set(LIB_NAME ilkeigenbackend)
add_library(${LIB_NAME} SHARED ${SOURCES})
install(FILES   ${HEADERS} DESTINATION ${HEADER_INSTALL_ROOT}/ilk/eigen)
install(TARGETS ${LIB_NAME} LIBRARY DESTINATION ${LIB_INSTALL_ROOT})

add_executable(test1   test1.cpp)