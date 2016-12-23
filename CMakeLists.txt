# Implementation of the GCPR 2016 Paper "Joint Object Pose Estimation and
# Shape Reconstruction in Urban Street Scenes Using 3D Shape Priors" by Engelmann et al.
# Copyright (C) 2016  Francis Engelmann - Visual Computing Institute RWTH Aachen University
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 2.8)
project(GCPR_shape_priors)

add_compile_options(-std=c++0x -fPIC -msse3 -Wall -fopenmp -Wno-unused-variable)
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules/" ${CMAKE_MODULE_PATH})

# Find Packages
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(VTK 7 REQUIRED)
include(${VTK_USE_FILE})
include_directories(${VTK_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

find_package(VIZ REQUIRED)
include_directories(${VIZ_INCLUDE_DIRS})

include_directories(src)
include_directories(scr/optimization)

file(GLOB SOURCES "${CMAKE_SOURCE_DIR}/src/*.cpp"
                  "${CMAKE_SOURCE_DIR}/src/*.h"
                  "${CMAKE_SOURCE_DIR}/src/optimization/*.cpp"
                  "${CMAKE_SOURCE_DIR}/src/optimization/*.h")

# MAC
if(APPLE)
  set(MORE_LIBS "pthread")

#LINUX
else(UNIX AND NOT APPLE)
  set(MORE_LIBS "pthread" "rt" "-fopenmp")
  set(CMAKE_MODULE_PATH "/usr/share/cmake-2.8/Modules" ${CMAKE_MODULE_PATH})
endif()

add_library(GVL SHARED ${SOURCES})
target_link_libraries(GVL ${OpenCV_LIBS} ${VTK_LIBRARIES} ${CERES_LIBRARIES} ${MORE_LIBS} ${VIZ_LIBRARIES})

# Actual apps -----------------------------------------------------------------------------------------------
add_executable(ShapePrior "apps/shapeprior.cpp" )
target_link_libraries(ShapePrior GVL ${OpenCV_LIBS} ${CERES_LIBRARIES} ${MORE_LIBS} ${VIZ_LIBRARIES}) #${VTK_LIBRARIES}
