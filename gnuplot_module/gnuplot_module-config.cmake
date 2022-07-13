include("${CMAKE_CURRENT_LIST_DIR}/gnuplot_module-targets.cmake")

get_target_property(gnuplot_module_INCLUDE_DIRS gnuplot_module::gnuplot_module INTERFACE_INCLUDE_DIRECTORIES)

get_property(gnuplot_module_LIB_CORE TARGET gnuplot_module::gnuplot_module PROPERTY LOCATION)
list(APPEND gnuplot_module_LIBRARIES ${gnuplot_module_LIB_CORE})
