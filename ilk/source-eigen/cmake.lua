local tpl = require('ilk.template-text')

local template = [[
cmake_minimum_required(VERSION 3.5)
project($(robot)-kingen)
find_package(PkgConfig)

# Eigen
pkg_search_module(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})
add_definitions(${EIGEN3_DEFINITIONS})

add_compile_options(-Wall -pedantic -std=c++11)

option(Coverage "Build for code coverage tests" OFF)
if(Coverage)
    add_compile_options(-g -O0 -fprofile-arcs -ftest-coverage)
    set(CMAKE_EXE_LINKER_FLAGS -lgcov)
else(Coverage)
    add_compile_options(-O3)
    add_definitions(-DEIGEN_NO_DEBUG)
endif(Coverage)


set(lib   "$(libname)")
set(libst "${lib}_st")

set(lib_srcs
    $(files.libsource).cpp
)

set(lib_headers
    $(files.libheader).h
    $(files.defsheader).h
)

# Build the shared and static library
add_library( ${lib} SHARED ${lib_srcs} )
target_link_libraries( ${lib} ilkeigenbackend )
add_library( ${libst} STATIC ${lib_srcs} )

# Install
# Relative destination paths are interpreted as relative to CMAKE_INSTALL_PREFIX
install(FILES   ${lib_headers}   DESTINATION include/$(includePath))
install(TARGETS ${lib}   LIBRARY DESTINATION lib)
install(TARGETS ${libst} ARCHIVE DESTINATION lib)


option(BuildTests "Build the test binaries" off)
if(BuildTests)
@for i,testfile in pairs(files.test) do
    add_executable($(testfile) $(testfile).cpp)
    target_link_libraries($(testfile) ${libst} ilkeigenbackend)
@end
endif(BuildTests)

]]

local function genText( args )
    local ok, res = tpl.template_eval(template, args, {verbose=true})
    if not ok then error(res) end
    return res
end

return { generator = genText }
