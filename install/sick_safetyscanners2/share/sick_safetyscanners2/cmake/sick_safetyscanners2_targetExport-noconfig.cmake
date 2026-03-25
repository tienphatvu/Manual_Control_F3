#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "sick_safetyscanners2::sick_safetyscanners2" for configuration ""
set_property(TARGET sick_safetyscanners2::sick_safetyscanners2 APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(sick_safetyscanners2::sick_safetyscanners2 PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libsick_safetyscanners2.so"
  IMPORTED_SONAME_NOCONFIG "libsick_safetyscanners2.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS sick_safetyscanners2::sick_safetyscanners2 )
list(APPEND _IMPORT_CHECK_FILES_FOR_sick_safetyscanners2::sick_safetyscanners2 "${_IMPORT_PREFIX}/lib/libsick_safetyscanners2.so" )

# Import target "sick_safetyscanners2::sick_safetyscanners2_lifecycle" for configuration ""
set_property(TARGET sick_safetyscanners2::sick_safetyscanners2_lifecycle APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(sick_safetyscanners2::sick_safetyscanners2_lifecycle PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libsick_safetyscanners2_lifecycle.so"
  IMPORTED_SONAME_NOCONFIG "libsick_safetyscanners2_lifecycle.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS sick_safetyscanners2::sick_safetyscanners2_lifecycle )
list(APPEND _IMPORT_CHECK_FILES_FOR_sick_safetyscanners2::sick_safetyscanners2_lifecycle "${_IMPORT_PREFIX}/lib/libsick_safetyscanners2_lifecycle.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
