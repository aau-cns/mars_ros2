
#function(build_mars_lib)

if (NOT TARGET mars_lib::mars_lib)

  include(ExternalProject)

  set(MarsLibrary_INSTALL_DIR "${CMAKE_BINARY_DIR}/mars_lib")

  externalproject_add(MarsLibrary_Project
      PREFIX ${CMAKE_BINARY_DIR}/mars_lib
      SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/mars_lib
      CMAKE_ARGS
      ${CROSS_COMPILE}
      -DOPTION_BUILD_DOCS=OFF
      -DOPTION_BUILD_EXAMPLES=OFF
      -DCMAKE_INSTALL_PREFIX=MarsLibrary_INSTALL_DIR
      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
      BUILD_ALWAYS 1
      )

  # Define include directories and library paths
  ExternalProject_Get_Property(MarsLibrary_Project install_dir)
  set(MARS_INCLUDE_DIR 
  "${install_dir}/include"
  "${install_dir}/src/mars_lib-ext-build/boost/include"
  "${install_dir}/src/mars_lib-ext-build/Eigen/include/eigen3"
  )
  set(MARS_LIBRARY_DIR "${install_dir}/lib")
  set(MARS_LIBRARIES "${MARS_LIBRARY_DIR}/libmars.a")

# Create imported target for mars_lib
add_library(mars_lib::mars_lib UNKNOWN IMPORTED)
add_dependencies(mars_lib::mars_lib MarsLibrary_Project)
set_target_properties(mars_lib::mars_lib PROPERTIES
IMPORTED_LOCATION ${MARS_LIBRARIES}
INTERFACE_INCLUDE_DIRECTORIES ${MARS_INCLUDE_DIR}
)

# Provide the variables to the calling scope
set(MARS_INCLUDE_DIRS ${MARS_INCLUDE_DIR} PARENT_SCOPE)
set(MARS_LIBRARIES mars_lib::mars_lib PARENT_SCOPE)


endif()

#endfunction()

#build_mars_lib()

# message("install_dir: ${install_dir}")
# message("MARS_INCLUDE_DIR: ${MARS_INCLUDE_DIR}")
# message("MARS_LIBRARY_DIR: ${MARS_LIBRARY_DIR}")
# message("MARS_LIBRARIES: ${MARS_LIBRARIES}")

# message("MARS_LIBRARIES: ${MARS_LIBRARIES}")
# message("MARS_INCLUDE_DIRS: ${MARS_INCLUDE_DIRS}")
# message( FATAL_ERROR "Stop" )

# set(MARS_INCLUDE_DIR 
# ${CMAKE_CURRENT_SOURCE_DIR}/mars_lib/source/mars/include
# ${CMAKE_CURRENT_BINARY_DIR}/mars_lib/src/mars_lib-ext-build/boost/include
# ${CMAKE_CURRENT_BINARY_DIR}/mars_lib/src/mars_lib-ext-build/Eigen/include/eigen3
# )
# set(MARS_LIBRARY_RELEASE ${CMAKE_BINARY_DIR}/mars_lib/lib/libmars.a)