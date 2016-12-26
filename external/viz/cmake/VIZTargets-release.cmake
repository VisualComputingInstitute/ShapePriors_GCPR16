#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "viz::VIZ" for configuration "Release"
set_property(TARGET viz::VIZ APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(viz::VIZ PROPERTIES
  IMPORTED_LOCATION_RELEASE "/Users/francis/Programming/GCPR16_shapepriors/external/viz/lib/libVIZ.0.0.1.dylib"
  IMPORTED_SONAME_RELEASE "@rpath/libVIZ.0.0.1.dylib"
  )

list(APPEND _IMPORT_CHECK_TARGETS viz::VIZ )
list(APPEND _IMPORT_CHECK_FILES_FOR_viz::VIZ "/Users/francis/Programming/GCPR16_shapepriors/external/viz/lib/libVIZ.0.0.1.dylib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
