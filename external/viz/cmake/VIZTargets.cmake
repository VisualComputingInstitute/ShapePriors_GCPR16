# Generated by CMake 3.6.1

if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 2.5)
   message(FATAL_ERROR "CMake >= 2.6.0 required")
endif()
cmake_policy(PUSH)
cmake_policy(VERSION 2.6)
#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Protect against multiple inclusion, which would fail when already imported targets are added once more.
set(_targetsDefined)
set(_targetsNotDefined)
set(_expectedTargets)
foreach(_expectedTarget viz::VIZ)
  list(APPEND _expectedTargets ${_expectedTarget})
  if(NOT TARGET ${_expectedTarget})
    list(APPEND _targetsNotDefined ${_expectedTarget})
  endif()
  if(TARGET ${_expectedTarget})
    list(APPEND _targetsDefined ${_expectedTarget})
  endif()
endforeach()
if("${_targetsDefined}" STREQUAL "${_expectedTargets}")
  unset(_targetsDefined)
  unset(_targetsNotDefined)
  unset(_expectedTargets)
  set(CMAKE_IMPORT_FILE_VERSION)
  cmake_policy(POP)
  return()
endif()
if(NOT "${_targetsDefined}" STREQUAL "")
  message(FATAL_ERROR "Some (but not all) targets in this export set were already defined.\nTargets Defined: ${_targetsDefined}\nTargets not yet defined: ${_targetsNotDefined}\n")
endif()
unset(_targetsDefined)
unset(_targetsNotDefined)
unset(_expectedTargets)


# The installation prefix configured by this project.
set(_IMPORT_PREFIX "/usr/local")

# Create imported target viz::VIZ
add_library(viz::VIZ SHARED IMPORTED)

set_target_properties(viz::VIZ PROPERTIES
  COMPATIBLE_INTERFACE_STRING "VIZ_MAJOR_VERSION"
  INTERFACE_LINK_LIBRARIES "vtkChartsCore;vtkCommonColor;vtkCommonDataModel;vtkCommonMath;vtkCommonCore;vtksys;vtkCommonMisc;vtkCommonSystem;vtkCommonTransforms;vtkInfovisCore;vtkFiltersExtraction;vtkCommonExecutionModel;vtkFiltersCore;vtkFiltersGeneral;vtkCommonComputationalGeometry;vtkFiltersStatistics;vtkImagingFourier;vtkImagingCore;vtkalglib;vtkRenderingContext2D;vtkRenderingCore;vtkFiltersGeometry;vtkFiltersSources;vtkRenderingFreeType;vtkfreetype;/usr/lib/libz.dylib;vtkDICOMParser;vtkDomainsChemistry;vtkIOXML;vtkIOGeometry;vtkIOCore;vtkIOXMLParser;/usr/lib/libexpat.dylib;vtkDomainsChemistryOpenGL2;vtkRenderingOpenGL2;vtkImagingHybrid;vtkIOImage;vtkmetaio;/usr/local/lib/libjpeg.dylib;/usr/local/lib/libpng.dylib;/usr/local/lib/libtiff.dylib;vtkglew;vtkFiltersAMR;vtkParallelCore;vtkIOLegacy;vtkFiltersFlowPaths;vtkFiltersGeneric;vtkFiltersHybrid;vtkImagingSources;vtkFiltersHyperTree;vtkFiltersImaging;vtkImagingGeneral;vtkFiltersModeling;vtkFiltersParallel;vtkFiltersParallelImaging;vtkFiltersProgrammable;vtkFiltersPython;/System/Library/Frameworks/Python.framework/Versions/2.7/lib/libpython2.7.dylib;vtkWrappingPythonCore;vtkWrappingTools;vtkFiltersSMP;vtkFiltersSelection;vtkFiltersTexture;vtkFiltersVerdict;verdict;vtkGeovisCore;vtkInfovisLayout;vtkInteractionStyle;vtkInteractionWidgets;vtkRenderingAnnotation;vtkImagingColor;vtkRenderingVolume;vtkViewsCore;vtkproj4;vtkIOAMR;/usr/local/lib/libhdf5.dylib;/usr/local/lib/libsz.dylib;/usr/lib/libdl.dylib;/usr/lib/libm.dylib;vtkIOEnSight;vtkIOExodus;vtkexoIIc;vtkNetCDF;vtkNetCDF_cxx;vtkIOExport;vtkRenderingLabel;vtkIOImport;vtkIOInfovis;/usr/lib/libxml2.dylib;vtkIOLSDyna;vtkIOMINC;vtkIOMovie;vtkoggtheora;vtkIONetCDF;vtkIOPLY;vtkIOParallel;vtkjsoncpp;vtkIOParallelXML;vtkIOSQL;vtksqlite;vtkIOVideo;vtkImagingMath;vtkImagingMorphological;vtkImagingStatistics;vtkImagingStencil;vtkInfovisBoostGraphAlgorithms;vtkInteractionImage;vtkRenderingContextOpenGL2;vtkRenderingFreeTypeFontConfig;vtkRenderingImage;vtkRenderingLOD;vtkRenderingVolumeOpenGL2;vtkViewsContext2D;vtkViewsInfovis;opencv_videostab;opencv_video;opencv_ts;opencv_superres;opencv_stitching;opencv_photo;opencv_ocl;opencv_objdetect;opencv_nonfree;opencv_ml;opencv_legacy;opencv_imgproc;opencv_highgui;opencv_gpu;opencv_flann;opencv_features2d;opencv_core;opencv_contrib;opencv_calib3d"
  INTERFACE_VIZ_MAJOR_VERSION "3"
)

if(CMAKE_VERSION VERSION_LESS 2.8.12)
  message(FATAL_ERROR "This file relies on consumers using CMake 2.8.12 or greater.")
endif()

# Load information for each installed configuration.
get_filename_component(_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
file(GLOB CONFIG_FILES "${_DIR}/VIZTargets-*.cmake")
foreach(f ${CONFIG_FILES})
  include(${f})
endforeach()

# Cleanup temporary variables.
set(_IMPORT_PREFIX)

# Loop over all imported files and verify that they actually exist
foreach(target ${_IMPORT_CHECK_TARGETS} )
  foreach(file ${_IMPORT_CHECK_FILES_FOR_${target}} )
    if(NOT EXISTS "${file}" )
      message(FATAL_ERROR "The imported target \"${target}\" references the file
   \"${file}\"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   \"${CMAKE_CURRENT_LIST_FILE}\"
but not all the files it references.
")
    endif()
  endforeach()
  unset(_IMPORT_CHECK_FILES_FOR_${target})
endforeach()
unset(_IMPORT_CHECK_TARGETS)

# This file does not depend on other imported targets which have
# been exported from the same project but in a separate export set.

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
cmake_policy(POP)
