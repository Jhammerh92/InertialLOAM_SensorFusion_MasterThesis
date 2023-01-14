#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "pclomp::registration" for configuration ""
set_property(TARGET pclomp::registration APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(pclomp::registration PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libregistration.so"
  IMPORTED_SONAME_NOCONFIG "libregistration.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS pclomp::registration )
list(APPEND _IMPORT_CHECK_FILES_FOR_pclomp::registration "${_IMPORT_PREFIX}/lib/libregistration.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
