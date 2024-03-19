cmake_minimum_required(VERSION 3.0)
project(ilk_eigen_backend)

find_package(PkgConfig)
pkg_check_modules(EIGEN3 eigen3 REQUIRED)

# Flags
set(CMAKE_CXX_FLAGS "-g -Wall -O3 -D EIGEN_NO_DEBUG -std=c++11")
include_directories(${EIGEN3_INCLUDE_DIRS})


set(HEADERS
  ./core-types.h
  ./joint-transforms.h
  ./ctransforms.h
  ./gjac.h
  ./rots.h
  ./ik.h
  ./misc.h
  ./operators.h
  ./test-utils.h
)
set(SOURCES
  ./joint-transforms.cpp
  ./ctransforms.cpp
  ./test-utils.cpp
)

#
# Add library and install targets
#
set(LIB_NAME ilkeigenbackend)
add_library(${LIB_NAME} SHARED ${SOURCES})
install(FILES   ${HEADERS} DESTINATION include/ilk/eigen)
install(TARGETS ${LIB_NAME} LIBRARY DESTINATION lib/)

add_executable(test1   test1.cpp)
