#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "gnuplot_module::gnuplot_module" for configuration ""
set_property(TARGET gnuplot_module::gnuplot_module APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(gnuplot_module::gnuplot_module PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libgnuplot_module.so"
  IMPORTED_SONAME_NOCONFIG "libgnuplot_module.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS gnuplot_module::gnuplot_module )
list(APPEND _IMPORT_CHECK_FILES_FOR_gnuplot_module::gnuplot_module "${_IMPORT_PREFIX}/lib/libgnuplot_module.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
