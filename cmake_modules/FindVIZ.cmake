# - Try to find VIZ
# Once done this will define
#  VIZ_FOUND - System has VIZ
#  VIZ_INCLUDE_DIRS - The VIZ include directories
#  VIZ_LIBRARIES - The libraries needed to use VIZ

find_path(VIZ_INCLUDE_DIR "viz" HINTS "${PROJECT_SOURCE_DIR}/external/viz/include/" )
find_library(VIZ_LIBRARY NAMES VIZ HINTS "${PROJECT_SOURCE_DIR}/external/viz/lib/")

set(VIZ_LIBRARIES ${VIZ_LIBRARY} )
set(VIZ_INCLUDE_DIRS ${VIZ_INCLUDE_DIR} )
