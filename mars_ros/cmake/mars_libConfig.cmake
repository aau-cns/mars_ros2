
function(build_mars_lib)

if (NOT TARGET mars_lib)
  include(ExternalProject)
  externalproject_add(mars_lib-ext
      PREFIX ${CMAKE_BINARY_DIR}/mars_lib
      SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/mars_lib
      CMAKE_ARGS
      ${CROSS_COMPILE}
      -DOPTION_BUILD_DOCS=OFF
      -DOPTION_BUILD_EXAMPLES=OFF
      -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/mars_lib
      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
      BUILD_ALWAYS 1
      )
  add_library(mars_lib INTERFACE IMPORTED GLOBAL)
  add_dependencies(mars_lib mars_lib-ext)
  file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/mars_lib)

endif()

endfunction()

build_mars_lib()

set(MARS_INCLUDE_DIRS
${CMAKE_CURRENT_SOURCE_DIR}/mars_lib/source/mars/include
${CMAKE_CURRENT_BINARY_DIR}/mars_lib/src/mars_lib-ext-build/boost/include
${CMAKE_CURRENT_BINARY_DIR}/mars_lib/src/mars_lib-ext-build/Eigen/include/eigen3
)

set(MARS_LIBRARY ${CMAKE_BINARY_DIR}/mars_lib/lib/libmars.a
)