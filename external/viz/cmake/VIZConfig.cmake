include("${CMAKE_CURRENT_LIST_DIR}/VIZTargets.cmake")

find_path(VIZ_INCLUDE_DIR "viz" HINTS "${CMAKE_SOURCE_DIR}/include/" )
find_library(VIZ_LIBRARY NAMES VIZ HINTS "${CMAKE_SOURCE_DIR}/lib/")

set(VIZ_LIBRARIES ${VIZ_LIBRARY} )
set(VIZ_INCLUDE_DIRS ${VIZ_INCLUDE_DIR} )


# pointcloud
# boudningbox
