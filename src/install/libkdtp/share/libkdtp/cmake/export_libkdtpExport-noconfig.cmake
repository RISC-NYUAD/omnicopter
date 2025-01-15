#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "libkdtp::libkdtp" for configuration ""
set_property(TARGET libkdtp::libkdtp APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(libkdtp::libkdtp PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/liblibkdtp.so"
  IMPORTED_SONAME_NOCONFIG "liblibkdtp.so"
  )

list(APPEND _cmake_import_check_targets libkdtp::libkdtp )
list(APPEND _cmake_import_check_files_for_libkdtp::libkdtp "${_IMPORT_PREFIX}/lib/liblibkdtp.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
