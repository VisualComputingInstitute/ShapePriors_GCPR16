# MIT License
#
# Copyright (c) 2016 Francis Engelmann
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

cmake_minimum_required(VERSION 3.0)
project(viz C CXX)
set(VIZ_VERSION 0.0.1)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_INCLUDE_CURRENT_DIR_IN_INTERFACE ON)

# Compiler options
add_compile_options(-std=c++0x -fPIC -msse3 -Wall -fopenmp -Wno-unused-variable)

# Set CMAKE_MODULE_PATH
#set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules/" ${CMAKE_MODULE_PATH})

# VTK required
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
include_directories(${VTK_INCLUDE_DIRS})

# Eigen required
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(OpenCV REQUIRED)

# Create Library VIZ
file(GLOB SOURCES "${CMAKE_SOURCE_DIR}/src/*.cpp")
file(GLOB HEADERS "${CMAKE_SOURCE_DIR}/src/*.h")
add_subdirectory(src)
add_library(VIZ SHARED ${SOURCES})
target_link_libraries(VIZ ${VTK_LIBRARIES} ${OpenCV_LIBS})

# Set Properties
include(GenerateExportHeader)
generate_export_header(VIZ)
set_property(TARGET VIZ PROPERTY VERSION ${VIZ_VERSION})
#set_property(TARGET VIZ PROPERTY SOVERSION 3)
set_property(TARGET VIZ PROPERTY INTERFACE_VIZ_MAJOR_VERSION 3)
set_property(TARGET VIZ APPEND PROPERTY
  COMPATIBLE_INTERFACE_STRING VIZ_MAJOR_VERSION
)

#install(TARGETS VIZ EXPORT VIZTargets
#        DESTINATION "${CMAKE_SOURCE_DIR}/lib")

# Install things
install(TARGETS VIZ EXPORT VIZTargets
  LIBRARY DESTINATION "${CMAKE_SOURCE_DIR}/lib"
  ARCHIVE DESTINATION "${CMAKE_SOURCE_DIR}/lib"
  RUNTIME DESTINATION "${CMAKE_SOURCE_DIR}/bin"
  #INCLUDES DESTINATION "${CMAKE_SOURCE_DIR}/include/viz"
)

install(FILES ${HEADERS}
        DESTINATION "${CMAKE_SOURCE_DIR}/include/viz"
        COMPONENT Devel
)

# CMAKE VIZConfig.cmake
include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/viz/VIZConfigVersion.cmake"
  VERSION ${VIZ_VERSION}
  COMPATIBILITY AnyNewerVersion
)

export(EXPORT VIZTargets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/viz/VIZTargets.cmake"
  NAMESPACE viz::
)

configure_file("${CMAKE_SOURCE_DIR}/cmake/VIZConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/viz/VIZConfig.cmake"
  #COPY_ONLY
)

set(ConfigPackageLocation "${CMAKE_SOURCE_DIR}/cmake")

install(EXPORT VIZTargets
  FILE VIZTargets.cmake
  NAMESPACE viz::
  DESTINATION ${ConfigPackageLocation}
)

install(FILES cmake/VIZConfig.cmake "${CMAKE_CURRENT_BINARY_DIR}/viz/VIZConfigVersion.cmake"
        DESTINATION ${ConfigPackageLocation}
        COMPONENT Devel
)

# Setting the build type
if (NOT CMAKE_BUILD_TYPE)
  message("-- No build type specified; defaulting to CMAKE_BUILD_TYPE=Release.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
else (NOT CMAKE_BUILD_TYPE)
  if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("\n=================================================================================")
    message("\n-- Build type: Debug. Performance will be terrible!")
    message("-- Add -DCMAKE_BUILD_TYPE=Release to the CMake command line to get an optimized build.")
    message("\n=================================================================================")
  endif (CMAKE_BUILD_TYPE STREQUAL "Debug")
endif (NOT CMAKE_BUILD_TYPE)

# Build the examples
option(BUILD_EXAMPLES "Build examples" ON)
if (BUILD_EXAMPLES)
  message("-- Build the examples.")
  add_subdirectory(examples)
else (BUILD_EXAMPLES)
  message("-- Do not build any example.")
endif (BUILD_EXAMPLES)
